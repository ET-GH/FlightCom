#include "usb_comm.h"

#include "main.h"

/*
 * Use the CubeMX-generated CDC application header.
 *
 * It includes ux_api.h first, which defines UINT, UCHAR, UX_NULL,
 * UX_STATE_NEXT, UX_STATE_EXIT, and the other USBX base definitions.
 * It then includes the CDC ACM class header.
 */
#include "ux_device_cdc_acm.h"

#include <stddef.h>
#include <string.h>

#if defined(__GNUC__)
#define USB_COMM_WEAK __attribute__((weak))
#else
#define USB_COMM_WEAK
#endif

/*
 * Full-Speed USB CDC endpoints use 64-byte packets.
 *
 * AMBAR encoded frames can be up to 66 bytes, so a 256-byte receive
 * ring easily holds several USB packets while the protocol layer catches up.
 */
#define USB_COMM_RX_TRANSFER_SIZE    64U
#define USB_COMM_RX_RING_SIZE        256U
#define USB_COMM_TX_STAGE_SIZE       AMBAR_HIL_USB_MAX_FRAME_SIZE
/*
 * Additive direct-USB command. This local constant avoids changing any existing
 * wire value; keep it synchronized with RawImportController.
 */
#define USB_COMM_COMMAND_EXPORT_ARCHIVE 0x33U

/*
 * main.c supplies the strong implementation. The weak fallback keeps this
 * transport module linkable in configurations that omit flash export.
 */
__weak uint8_t USBComm_ApplicationRequestArchiveExport(
    uint16_t request_sequence,
    uint16_t *detail)
{
    (void)request_sequence;

    if (detail != NULL)
    {
        *detail = 0U;
    }

    return AMBAR_HIL_USB_ACK_UNSUPPORTED;
}


/*
 * USBX CDC class instance supplied by USBD_CDC_ACM_Activate().
 */
static UX_SLAVE_CLASS_CDC_ACM *g_usb_cdc = UX_NULL;

/*
 * AMBAR protocol context.
 */
static AmbarHilUsb g_ambar_usb;
static bool g_ambar_usb_initialized = false;

/*
 * USBX standalone receive state.
 */
static UCHAR g_usb_rx_transfer[USB_COMM_RX_TRANSFER_SIZE];
static ULONG g_usb_rx_actual_length = 0U;

/*
 * Raw bytes completed by USBX but not yet consumed by AMBAR.
 */
static uint8_t g_usb_rx_ring[USB_COMM_RX_RING_SIZE];
static size_t g_usb_rx_head = 0U;
static size_t g_usb_rx_tail = 0U;
static size_t g_usb_rx_count = 0U;

/*
 * USBX standalone write state.
 *
 * AMBAR copies one encoded frame here. USBX then transmits from this
 * persistent buffer over as many main-loop iterations as required.
 */
static UCHAR g_usb_tx_stage[USB_COMM_TX_STAGE_SIZE];
static ULONG g_usb_tx_length = 0U;
static ULONG g_usb_tx_actual_length = 0U;
static bool g_usb_tx_pending = false;

/*
 * Diagnostic counters. These can be added to Live Expressions.
 */
volatile uint32_t g_usb_comm_rx_overflows = 0U;
volatile uint32_t g_usb_comm_rx_errors = 0U;
volatile uint32_t g_usb_comm_tx_errors = 0U;

/* ------------------------------------------------------------------------- */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------- */

static void USBComm_WriteU16LE(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void USBComm_WriteU32LE(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void USBComm_ResetBuffers(void)
{
    g_usb_rx_actual_length = 0U;

    g_usb_rx_head = 0U;
    g_usb_rx_tail = 0U;
    g_usb_rx_count = 0U;

    g_usb_tx_length = 0U;
    g_usb_tx_actual_length = 0U;
    g_usb_tx_pending = false;
}

static bool USBComm_CdcConfigured(void)
{
    if ((g_usb_cdc == UX_NULL) ||
        (_ux_system_slave == UX_NULL))
    {
        return false;
    }

    return
        _ux_system_slave
            ->ux_system_slave_device
            .ux_slave_device_state == UX_DEVICE_CONFIGURED;
}

static void USBComm_PushReceivedBytes(
    const uint8_t *data,
    size_t length)
{
    size_t index;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        if (g_usb_rx_count >= USB_COMM_RX_RING_SIZE)
        {
            /*
             * Do not overwrite unread data. The AMBAR decoder will recover
             * at the next zero-byte frame delimiter.
             */
            ++g_usb_comm_rx_overflows;
            break;
        }

        g_usb_rx_ring[g_usb_rx_tail] = data[index];
        g_usb_rx_tail =
            (g_usb_rx_tail + 1U) % USB_COMM_RX_RING_SIZE;
        ++g_usb_rx_count;
    }
}

/* ------------------------------------------------------------------------- */
/* AMBAR I/O callbacks                                                       */
/* ------------------------------------------------------------------------- */

static size_t USBComm_AmbarRead(
    void *user,
    uint8_t *destination,
    size_t capacity)
{
    size_t copied = 0U;

    (void)user;

    if ((destination == NULL) || (capacity == 0U))
    {
        return 0U;
    }

    while ((copied < capacity) && (g_usb_rx_count != 0U))
    {
        destination[copied] = g_usb_rx_ring[g_usb_rx_head];

        g_usb_rx_head =
            (g_usb_rx_head + 1U) % USB_COMM_RX_RING_SIZE;

        --g_usb_rx_count;
        ++copied;
    }

    return copied;
}

static size_t USBComm_AmbarWrite(
    void *user,
    const uint8_t *source,
    size_t length)
{
    (void)user;

    if ((source == NULL) && (length != 0U))
    {
        return AMBAR_HIL_USB_IO_ERROR;
    }

    if (!USBComm_CdcConfigured())
    {
        return AMBAR_HIL_USB_IO_ERROR;
    }

    if (length == 0U)
    {
        return 0U;
    }

    if (length > sizeof(g_usb_tx_stage))
    {
        return AMBAR_HIL_USB_IO_ERROR;
    }

    /*
     * USBX is still transmitting the previous staged frame.
     * Returning zero tells AMBAR to retry this frame later.
     */
    if (g_usb_tx_pending)
    {
        return 0U;
    }

    memcpy(g_usb_tx_stage, source, length);

    g_usb_tx_length = (ULONG)length;
    g_usb_tx_actual_length = 0U;
    g_usb_tx_pending = true;

    /*
     * The bytes have been accepted into persistent storage. USBX will
     * transmit them asynchronously from USBComm_ServiceTransmit().
     */
    return length;
}

static uint32_t USBComm_AmbarTime(void *user)
{
    (void)user;
    return HAL_GetTick();
}

static bool USBComm_AmbarConnected(void *user)
{
    (void)user;
    return USBComm_CdcConfigured();
}

/* ------------------------------------------------------------------------- */
/* USBX standalone transfer state machines                                   */
/* ------------------------------------------------------------------------- */

static void USBComm_ServiceReceive(void)
{
    UINT state;

    if (!USBComm_CdcConfigured())
    {
        g_usb_rx_actual_length = 0U;
        return;
    }

    state = ux_device_class_cdc_acm_read_run(
        g_usb_cdc,
        g_usb_rx_transfer,
        sizeof(g_usb_rx_transfer),
        &g_usb_rx_actual_length);

    if (state == UX_STATE_NEXT)
    {
        if (g_usb_rx_actual_length != 0U)
        {
            USBComm_PushReceivedBytes(
                g_usb_rx_transfer,
                (size_t)g_usb_rx_actual_length);
        }

        g_usb_rx_actual_length = 0U;
    }
    else if ((state == UX_STATE_EXIT) ||
             (state == UX_STATE_ERROR))
    {
        ++g_usb_comm_rx_errors;
        g_usb_rx_actual_length = 0U;
    }
}

static void USBComm_ServiceTransmit(void)
{
    UINT state;

    if (!g_usb_tx_pending)
    {
        return;
    }

    if (!USBComm_CdcConfigured())
    {
        g_usb_tx_pending = false;
        g_usb_tx_length = 0U;
        g_usb_tx_actual_length = 0U;
        return;
    }

    state = ux_device_class_cdc_acm_write_run(
        g_usb_cdc,
        g_usb_tx_stage,
        g_usb_tx_length,
        &g_usb_tx_actual_length);

    if (state == UX_STATE_NEXT)
    {
        g_usb_tx_pending = false;
        g_usb_tx_length = 0U;
        g_usb_tx_actual_length = 0U;
    }
    else if ((state == UX_STATE_EXIT) ||
             (state == UX_STATE_ERROR))
    {
        ++g_usb_comm_tx_errors;

        g_usb_tx_pending = false;
        g_usb_tx_length = 0U;
        g_usb_tx_actual_length = 0U;
    }
}

/* ------------------------------------------------------------------------- */
/* Application command callback                                              */
/* ------------------------------------------------------------------------- */

USB_COMM_WEAK uint8_t USBComm_ExecuteCommand(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length,
    uint16_t command_sequence,
    uint16_t *detail)
{
    (void)command;
    (void)payload;
    (void)payload_length;
    (void)command_sequence;

    if (detail != NULL)
    {
        *detail = 0U;
    }

    /*
     * The safe fallback never reports an application command as successful
     * unless main.c supplies a strong implementation.
     */
    return AMBAR_HIL_USB_ACK_UNSUPPORTED;
}

/* ------------------------------------------------------------------------- */
/* Initial command handler                                                   */
/* ------------------------------------------------------------------------- */

static void USBComm_HandleMessage(
    const AmbarHilUsbMessage *message)
{
    AmbarHilUsbAck ack;

    if ((message == NULL) ||
        (message->kind != AMBAR_HIL_USB_RX_COMMAND))
    {
        return;
    }

    memset(&ack, 0, sizeof(ack));

    /*
     * The ACK references the sequence number of the received command.
     */
    ack.command_sequence = message->sequence;
    ack.command = message->body.command.command;
    ack.result = AMBAR_HIL_USB_ACK_UNSUPPORTED;
    ack.detail = 0U;

    switch (message->body.command.command)
    {
        case AMBAR_HIL_USB_COMMAND_PING:
        {
            if (message->body.command.payload_length != 0U)
            {
                ack.result = AMBAR_HIL_USB_ACK_BAD_LENGTH;
            }
            else
            {
                ack.result = AMBAR_HIL_USB_ACK_OK;
            }

            break;
        }
        case USB_COMM_COMMAND_EXPORT_ARCHIVE:
                {
                    if (message->body.command.payload_length != 0U)
                    {
                        ack.result = AMBAR_HIL_USB_ACK_BAD_LENGTH;
                    }
                    else
                    {
                        ack.result =
                            USBComm_ApplicationRequestArchiveExport(
                                message->sequence,
                                &ack.detail);
                    }

                    break;
                }

        default:
        {
            ack.result = USBComm_ExecuteCommand(
                message->body.command.command,
                message->body.command.payload,
                message->body.command.payload_length,
                message->sequence,
                &ack.detail);
            break;
        }
    }

    (void)AmbarHilUsb_SendAck(&g_ambar_usb, &ack);
}

/* ------------------------------------------------------------------------- */
/* Public interface                                                          */
/* ------------------------------------------------------------------------- */

bool USBComm_Init(void)
{
    const AmbarHilUsbIo io =
    {
        .read = USBComm_AmbarRead,
        .write = USBComm_AmbarWrite,
        .now_ms = USBComm_AmbarTime,
        .is_connected = USBComm_AmbarConnected,
        .user = NULL
    };

    USBComm_ResetBuffers();

    g_ambar_usb_initialized =
        AmbarHilUsb_Init(&g_ambar_usb, &io);

    return g_ambar_usb_initialized;
}

void USBComm_Attach(void *cdc_acm_instance)
{
    g_usb_cdc =
        (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

    USBComm_ResetBuffers();

    if (g_ambar_usb_initialized)
    {
        AmbarHilUsb_Reset(&g_ambar_usb);
    }
}

void USBComm_Detach(void)
{
    g_usb_cdc = UX_NULL;
    USBComm_ResetBuffers();
}

bool USBComm_IsConnected(void)
{
    return USBComm_CdcConfigured();
}

void USBComm_Task(void)
{
    AmbarHilUsbMessage message;

    if (!g_ambar_usb_initialized)
    {
        return;
    }

    /*
     * Advance at most one CDC receive and one CDC transmit state machine
     * per call. Neither operation blocks.
     */
    USBComm_ServiceReceive();
    USBComm_ServiceTransmit();

    /*
     * Decode newly received bytes and offer one queued AMBAR frame to
     * the CDC write adapter.
     */
    AmbarHilUsb_Poll(&g_ambar_usb);

    while (AmbarHilUsb_TakeMessage(
               &g_ambar_usb,
               &message))
    {
        USBComm_HandleMessage(&message);
    }
}

bool USBComm_SendTelemetry(
    const AmbarHilUsbTelemetry *telemetry)
{
    if (!g_ambar_usb_initialized)
    {
        return false;
    }

    return AmbarHilUsb_SendTelemetry(
        &g_ambar_usb,
        telemetry);
}

bool USBComm_SendEvent(
    const AmbarHilUsbEvent *event)
{
    if (!g_ambar_usb_initialized)
    {
        return false;
    }

    return AmbarHilUsb_SendEvent(
        &g_ambar_usb,
        event);
}

bool USBComm_SendHeartbeat(
    const AmbarHilUsbHeartbeat *heartbeat)
{
    if (!g_ambar_usb_initialized)
    {
        return false;
    }

    return AmbarHilUsb_SendHeartbeat(
        &g_ambar_usb,
        heartbeat);
}

bool USBComm_SendActuatorStatus(
    const AmbarHilUsbActuatorStatus *status)
{
    if (!g_ambar_usb_initialized)
    {
        return false;
    }

    return AmbarHilUsb_SendActuatorStatus(
        &g_ambar_usb,
        status);
}

bool USBComm_SendLogStatus(
    const AmbarHilUsbLogStatus *status)
{
    uint8_t payload[AMBAR_HIL_USB_LOG_STATUS_PAYLOAD_SIZE];

    if (!g_ambar_usb_initialized || (status == NULL))
    {
        return false;
    }

    USBComm_WriteU16LE(payload + 0U, status->command_sequence);
    payload[2] = status->state;
    payload[3] = status->error_code;
    USBComm_WriteU32LE(payload + 4U, status->total_records);
    USBComm_WriteU32LE(payload + 8U, status->records_sent);
    USBComm_WriteU32LE(payload + 12U, status->corrupt_records);

    return AmbarHilUsb_SendPacket(
        &g_ambar_usb,
        AMBAR_HIL_USB_PACKET_LOG_STATUS,
        payload,
        sizeof(payload));
}

AmbarHilUsbStats USBComm_GetStats(void)
{
    return AmbarHilUsb_GetStats(&g_ambar_usb);
}
