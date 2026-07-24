/**
 * @file radio_bridge.c
 * @brief SX1280 telemetry, event reporting, and ground-command bridge.
 *
 * The radio bridge converts BehaviorTelemetry_t into the stable wire format
 * defined by rocket_protocol.h. It also receives command packets, extracts and
 * validates their payloads, dispatches application commands to main.c, and
 * returns protocol acknowledgements.
 *
 * Normal operation consists of three independent activities:
 *   1. Receive and process pending command packets.
 *   2. Repeat event packets when monitored state changes.
 *   3. Transmit periodic or explicitly requested telemetry snapshots.
 *
 * Command interpretation is always active. Disabling either normal radio data
 * or the flight computer only suppresses telemetry/event traffic; command ACKs
 * are still transmitted so either subsystem can be enabled again remotely.
 */

#include "radio_bridge.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "airbrake.h"
#include "rocket_protocol.h"
#include "sx1280.h"

/* -------------------------------------------------------------------------- */
/* Timing and protocol constants                                              */
/* -------------------------------------------------------------------------- */

#define RADIO_TX_TIMEOUT_MS       2000U
#define TELEMETRY_PERIOD_MS       250U   /* 10 Hz normal telemetry stream. */
#define HEARTBEAT_PERIOD_MS       1000U
/*
 * Events are already protected by the packet CRC and carry sequence numbers.
 * Sending three blocking copies consecutively consumes radio receive time and
 * does not provide useful time diversity.
 */
#define EVENT_REPEAT_COUNT        1U

/*
 * Only stable, operator-relevant transitions generate event packets.
 *
 * The excluded flags remain available in normal telemetry:
 *
 * - BARO_VALID
 * - FUSION_VALID
 * - EKF_VALID
 * - CONTROLLER_ACTIVE
 * - MODE_CHANGED
 * - BARO_CORRECTION_USED
 * - FUSION_STARTUP
 * - ACCEL_IGNORED
 * - MAG_IGNORED
 *
 * These values may change during normal estimator operation and therefore
 * must not be treated as discrete flight events.
 */
#define RADIO_EVENT_FLAG_MASK                                      \
    ((uint16_t)(ROCKET_FLAG_INITIALIZED        |                    \
                ROCKET_FLAG_CONTROLLER_ENABLED |                    \
                ROCKET_FLAG_APOGEE_REACHED     |                    \
                ROCKET_FLAG_PIPELINE_COMPLETE  |                    \
                ROCKET_FLAG_PIPELINE_PASS      |                    \
                ROCKET_FLAG_SENSOR_FAULT))

/* Flight-phase values stored in the low nibble of the compact state byte. */
#define RADIO_PHASE_LAUNCH        1U
#define RADIO_PHASE_BURNOUT       2U
#define RADIO_PHASE_LANDED        4U

/* The command packet contains a 9-byte protocol header followed by command and
 * payload-length bytes. The payload itself is limited by RocketCommandPayload. */
#define RADIO_COMMAND_PREFIX_SIZE (ROCKET_PROTOCOL_HEADER_SIZE + 2U)

/* GCC is the compiler used by STM32CubeIDE. A weak default callback keeps the
 * bridge independently linkable while allowing main.c to replace it. */
#if defined(__GNUC__)
#define RADIO_BRIDGE_WEAK __attribute__((weak))
#else
#define RADIO_BRIDGE_WEAK
#endif

/* -------------------------------------------------------------------------- */
/* Multipart acknowledgement representation                                  */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t result;
    uint8_t detail_type;
    uint32_t detail;
} RadioBridgeAckPart;

/* -------------------------------------------------------------------------- */
/* Module state                                                               */
/* -------------------------------------------------------------------------- */

/** Set by the DIO1 external-interrupt callback and consumed by the task loop. */
static volatile uint8_t dio1_seen;

/** Sequence number assigned to each outgoing packet. */
static uint16_t tx_sequence;

static uint32_t last_telemetry_ms;
static uint32_t last_heartbeat_ms;

/** Previous values used to detect reportable telemetry transitions. */
static uint16_t previous_flags;
static uint8_t previous_state;
static uint8_t previous_status = 0xFFU;
static uint8_t previous_deployment;

/*
 * Prevent startup telemetry from being interpreted as a collection of state
 * changes. The first valid telemetry structure establishes the baseline.
 */
static bool event_baseline_valid;

/** Nonzero when the next allowed task iteration must send a snapshot. */
static uint8_t snapshot_requested;

/**
 * Normal data-transmission gates.
 *
 * radio_data_enabled implements ROCKET_SUBSYSTEM_RADIO. The separate flight-
 * computer gate prevents stale telemetry from being streamed while main.c has
 * suspended flight-computer logic. Neither gate applies to command ACKs.
 */
static bool radio_data_enabled;
static bool flight_computer_enabled;

/** One-byte, saturating link counters reported and reset by each PING. */
static uint8_t tx_packets_since_ping;
static uint8_t rx_packets_since_ping;
static uint8_t tx_errors_since_ping;
static uint8_t rx_errors_since_ping;

/**
 * Last packet signal values in protocol units.
 *
 * The supplied SX1280 driver interface does not expose packet RSSI/SNR, so the
 * values remain zero until that driver adds an accessor. The ground station
 * still reports its own measured RSSI/SNR for every received packet.
 */
static int16_t last_rssi_dbm_x10;
static int16_t last_snr_db_x100;

/*
 * Zero means no pending motor ACK.
 *
 * Bit 31 marks a pending command.
 * Bits 0 through 15 hold the original command sequence.
 */
static uint32_t pending_motor_ack;

/* -------------------------------------------------------------------------- */
/* Private function declarations                                              */
/* -------------------------------------------------------------------------- */

static void RadioBridge_IncrementSaturating(uint8_t *counter);
static bool RadioBridge_NormalDataAllowed(void);
static int16_t RadioBridge_ClampI16(float value);
static uint16_t RadioBridge_ClampU16(float value);
static bool RadioBridge_SensorStatusIsFault(HAL_StatusTypeDef status);
static uint16_t RadioBridge_MakeFlags(
    const volatile BehaviorTelemetry_t *telemetry);
static uint8_t RadioBridge_MakeState(
    const volatile BehaviorTelemetry_t *telemetry);
static uint8_t RadioBridge_MakeSensorHealth(
    const volatile BehaviorTelemetry_t *telemetry);
static uint8_t RadioBridge_MapStatus(uint32_t status);
static uint8_t RadioBridge_SelectEventMessage(
    const volatile BehaviorTelemetry_t *telemetry,
    uint16_t changed_flags,
    uint8_t old_state,
    uint8_t new_state,
    uint8_t old_deployment);

static bool RadioBridge_ExtractCommand(const uint8_t *packet,
                                       uint8_t length,
                                       RocketPacketHeader *header,
                                       RocketCommandPayload *command,
                                       uint8_t *extract_result);
static uint8_t RadioBridge_ValidateCommand(
    const RocketCommandPayload *command,
    uint32_t *detail);
static uint8_t RadioBridge_DispatchCommand(
    const RocketCommandPayload *command,
    uint32_t now_ms,
    uint32_t *detail);
static void RadioBridge_ProcessCommand(const uint8_t *packet,
                                       uint8_t length,
                                       uint32_t now_ms);

static HAL_StatusTypeDef RadioBridge_Transmit(const uint8_t *packet,
                                               uint8_t length);
static HAL_StatusTypeDef RadioBridge_SendTelemetry(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    uint8_t message);
static HAL_StatusTypeDef RadioBridge_SendEvent(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    uint16_t changed_flags,
    uint8_t old_state,
    uint8_t message);
static void RadioBridge_SendAck(uint16_t command_sequence,
                                uint8_t command,
                                uint8_t result,
                                uint8_t detail_type,
                                uint8_t part_index,
                                uint8_t part_count,
                                uint32_t detail,
                                uint32_t now_ms);
static void RadioBridge_SendAckParts(uint16_t command_sequence,
                                     uint8_t command,
                                     const RadioBridgeAckPart *parts,
                                     uint8_t part_count,
                                     uint32_t now_ms);
static void RadioBridge_SendPingAcks(uint16_t command_sequence,
                                     uint32_t now_ms);
static void RadioBridge_SendDiagnosticAcks(
    uint16_t command_sequence,
    const RocketCommandPayload *command,
    uint32_t now_ms);

/* ========================================================================== */
/* Application command callback                                               */
/* ========================================================================== */

/**
 * @brief Execute a validated application command.
 *
 * main.c supplies a strong definition of this function. Keeping command packet
 * extraction and validation in the bridge prevents main.c from depending on
 * the wire layout, while main.c remains responsible for application state and
 * cross-subsystem policy.
 *
 * @param command Protocol command code.
 * @param payload Validated command payload.
 * @param payload_length Number of valid bytes in payload.
 * @param now_ms Current system timestamp in milliseconds.
 * @param detail Receives command-specific ACK detail.
 * @return One RocketAckCode value.
 */
RADIO_BRIDGE_WEAK uint8_t RadioBridge_ExecuteCommand(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length,
    uint32_t now_ms,
    uint32_t *detail)
{
    /* The weak fallback is intentionally safe: no command is reported as
     * successful unless the application has provided an implementation. */
    (void)command;
    (void)payload;
    (void)payload_length;
    (void)now_ms;

    if (detail != NULL)
    {
        *detail = 0U;
    }

    return ROCKET_ACK_UNSUPPORTED;
}

/* ========================================================================== */
/* Interrupt and public lifecycle API                                         */
/* ========================================================================== */

/**
 * @brief Record an SX1280 DIO1 interrupt for deferred processing.
 *
 * No SPI work is performed in interrupt context. RadioBridge_Task() reads the
 * packet during the next normal task iteration.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA_DIO1_Pin)
    {
        dio1_seen = 1U;
    }
}

/**
 * @brief Initialize the SX1280 and reset radio-bridge bookkeeping.
 *
 * The first RadioBridge_Task() call sends a telemetry snapshot because
 * snapshot_requested is initialized to one.
 */
HAL_StatusTypeDef RadioBridge_Init(void)
{
    HAL_StatusTypeDef status = SX1280_InitLoRa();

    dio1_seen = 0U;
    tx_sequence = 0U;
    last_telemetry_ms = 0U;
    last_heartbeat_ms = 0U;

    previous_flags = 0U;
    previous_state = 0xFFU;
    previous_status = 0xFFU;
    previous_deployment = 0U;

    event_baseline_valid = false;

    snapshot_requested = 1U;
    radio_data_enabled = true;
    flight_computer_enabled = true;

    tx_packets_since_ping = 0U;
    rx_packets_since_ping = 0U;
    tx_errors_since_ping = 0U;
    rx_errors_since_ping = 0U;
    last_rssi_dbm_x10 = 0;
    last_snr_db_x100 = 0;

    pending_motor_ack = 0U;

    return status;
}

/** @brief Request an immediate telemetry snapshot on the next allowed task. */
void RadioBridge_RequestSnapshot(void)
{
    snapshot_requested = 1U;
}

/* ========================================================================== */
/* Standard radio task                                                        */
/* ========================================================================== */

/**
 * @brief Service radio reception, event detection, and telemetry transmission.
 *
 * @param telemetry Current behavior telemetry. A NULL pointer still permits
 *                  command reception, but suppresses event/telemetry output.
 * @param now_ms Current system timestamp in milliseconds.
 */
void RadioBridge_Task(const volatile BehaviorTelemetry_t *telemetry,
                      uint32_t now_ms)
{
    uint8_t receive_buffer[ROCKET_PROTOCOL_MAX_PACKET];
    uint8_t receive_length = 0U;

    /* DIO1 is normally interrupt-driven, but polling the pin also catches an
     * event that occurred while interrupts were briefly unavailable. */
    if (dio1_seen ||
        (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) ==
         GPIO_PIN_SET))
    {
        HAL_StatusTypeDef receive_status;

        dio1_seen = 0U;
        receive_status = SX1280_ReadPacketIfAvailable(receive_buffer,
                                                       &receive_length);

        if (receive_status == HAL_OK)
        {
            RadioBridge_IncrementSaturating(&rx_packets_since_ping);
            RadioBridge_ProcessCommand(receive_buffer,
                                       receive_length,
                                       now_ms);
        }
        else if (receive_status != HAL_BUSY)
        {
            /* HAL_BUSY means that no complete packet was ready and is not a
             * link failure. All other unsuccessful reads count as RX errors. */
            RadioBridge_IncrementSaturating(&rx_errors_since_ping);
        }
    }

    /*
     * Service a deferred motor acknowledgement before applying telemetry gates.
     * This ensures motor completion and its ACK continue during standby, radio-data
     * disable, or flight-computer disable.
     */
    if (pending_motor_ack != 0U)
    {
        uint8_t poll_payload = 0U;
        uint32_t detail = 0U;
        uint8_t result;

        result = RadioBridge_ExecuteCommand(
            ROCKET_CMD_MOTOR_STEPS,
            &poll_payload,
            0U,                 /* Zero length means internal completion poll. */
            now_ms,
            &detail);

        if (result != ROCKET_ACK_BUSY)
        {
            const uint16_t command_sequence =
                (uint16_t)pending_motor_ack;

            pending_motor_ack = 0U;

            RadioBridge_SendAck(command_sequence,
                                ROCKET_CMD_MOTOR_STEPS,
                                result,
                                ROCKET_ACK_DETAIL_NONE,
                                0U,
                                1U,
                                detail,
                                now_ms);
        }
    }

    /* Command interpretation above is intentionally independent of telemetry.
     * The remaining work requires both a telemetry source and enabled data. */
    if ((telemetry == NULL) || !RadioBridge_NormalDataAllowed())
    {
        return;
    }

    {
        const uint16_t flags =
            RadioBridge_MakeFlags(telemetry);

        const uint8_t state =
            RadioBridge_MakeState(telemetry);

        const uint8_t status =
            RadioBridge_MapStatus((uint32_t)telemetry->status);

        const uint8_t deployment =
            telemetry->controller_requested_percent;

        /*
         * Establish a clean initial baseline.
         *
         * Startup state belongs in the first telemetry snapshot. It is not treated
         * as a collection of transitions from artificial zero/0xFF values.
         */
        if (!event_baseline_valid)
        {
            previous_flags = flags;
            previous_state = state;
            previous_status = status;
            previous_deployment = deployment;
            event_baseline_valid = true;
        }
        else
        {
            const uint16_t all_changed_flags =
                (uint16_t)(flags ^ previous_flags);

            /*
             * Only stable, flight-significant flag transitions are events.
             * Transient estimator and sensor-use indicators remain in telemetry.
             */
            const uint16_t event_changed_flags =
                (uint16_t)(all_changed_flags &
                           RADIO_EVENT_FLAG_MASK);

            const bool state_changed =
                (state != previous_state);

            const bool status_changed =
                (status != previous_status);

            /*
             * Deployment is an event only when crossing the retracted boundary:
             *
             *     0% -> nonzero: deployed
             *     nonzero -> 0%: retracted
             *
             * Normal percentage modulation remains ordinary telemetry.
             */
            const bool deployment_edge =
                ((previous_deployment == 0U) !=
                 (deployment == 0U));

            const uint8_t message =
                RadioBridge_SelectEventMessage(
                    telemetry,
                    event_changed_flags,
                    previous_state,
                    state,
                    previous_deployment);

            if ((event_changed_flags != 0U) ||
                state_changed ||
                status_changed ||
                deployment_edge)
            {
                uint8_t repeat;

                for (repeat = 0U;
                     repeat < EVENT_REPEAT_COUNT;
                     ++repeat)
                {
                    (void)RadioBridge_SendEvent(
                        telemetry,
                        now_ms,
                        event_changed_flags,
                        previous_state,
                        message);
                }
            }

            /*
             * Always advance the comparison baseline, including when only ignored
             * diagnostic flags changed.
             *
             * This prevents transient telemetry differences from accumulating and
             * appearing in an unrelated later event.
             */
            previous_flags = flags;
            previous_state = state;
            previous_status = status;
            previous_deployment = deployment;
        }
    }

    if (snapshot_requested ||
        ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS))
    {
        const uint8_t message = snapshot_requested
            ? ROCKET_MSG_SNAPSHOT
            : ROCKET_MSG_NONE;

        snapshot_requested = 0U;
        last_telemetry_ms = now_ms;

        (void)RadioBridge_SendTelemetry(telemetry, now_ms, message);
    }

    if ((uint32_t)(now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
    {
        last_heartbeat_ms = now_ms;

        /* The periodic binary telemetry stream doubles as the heartbeat, so no
         * additional heartbeat-only packet is transmitted here. */
    }
}

/* ========================================================================== */
/* Incoming command extraction, validation, and dispatch                      */
/* ========================================================================== */

/**
 * @brief Extract a command from one complete wire packet.
 *
 * Header/type failures are discarded because the packet cannot be trusted as a
 * command. Once the command byte is available, malformed payload sizing is
 * returned through extract_result so the sender receives BAD_LENGTH.
 */
static bool RadioBridge_ExtractCommand(const uint8_t *packet,
                                       uint8_t length,
                                       RocketPacketHeader *header,
                                       RocketCommandPayload *command,
                                       uint8_t *extract_result)
{
    size_t expected_length;

    if ((packet == NULL) ||
        (header == NULL) ||
        (command == NULL) ||
        (extract_result == NULL) ||
        !RocketProtocol_DecodeHeader(packet, length, header) ||
        (header->type != ROCKET_PKT_COMMAND) ||
        (length < RADIO_COMMAND_PREFIX_SIZE))
    {
        return false;
    }

    command->command = packet[ROCKET_PROTOCOL_HEADER_SIZE];
    command->payload_length = packet[ROCKET_PROTOCOL_HEADER_SIZE + 1U];
    memset(command->payload, 0, sizeof(command->payload));

    expected_length = RADIO_COMMAND_PREFIX_SIZE + command->payload_length;

    if ((command->payload_length > sizeof(command->payload)) ||
        (expected_length != length))
    {
        *extract_result = ROCKET_ACK_BAD_LENGTH;
        return true;
    }

    if (command->payload_length > 0U)
    {
        memcpy(command->payload,
               packet + RADIO_COMMAND_PREFIX_SIZE,
               command->payload_length);
    }

    *extract_result = ROCKET_ACK_OK;
    return true;
}

/**
 * @brief Validate command-specific payload length and value ranges.
 *
 * No subsystem function is called here. This keeps malformed commands from
 * partially changing state before the complete command is known to be valid.
 */
static uint8_t RadioBridge_ValidateCommand(
    const RocketCommandPayload *command,
    uint32_t *detail)
{
    if ((command == NULL) || (detail == NULL))
    {
        return ROCKET_ACK_EXECUTION_ERROR;
    }

    *detail = 0U;

    switch (command->command)
    {
        case ROCKET_CMD_NOP:
        case ROCKET_CMD_PING:
        case ROCKET_CMD_REQUEST_SNAPSHOT:
        case ROCKET_CMD_RETURN_STANDARD:
        case ROCKET_CMD_CLEAR_FAULTS:
        case ROCKET_CMD_CLEAR_MEMORY:
            return (command->payload_length == 0U)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_LENGTH;

        case ROCKET_CMD_SET_TARGET_APOGEE:
        case ROCKET_CMD_MOTOR_STEPS:
            return (command->payload_length == 2U)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_LENGTH;

        case ROCKET_CMD_SET_CONTROLLER:
            if (command->payload_length != 1U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }
            *detail = command->payload[0];
            return (command->payload[0] <= 1U)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_VALUE;

        case ROCKET_CMD_SET_MODE:
            if (command->payload_length != 3U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }
            *detail = command->payload[0];
            return (command->payload[0] <= ROCKET_MODE_TEST)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_VALUE;

        case ROCKET_CMD_MANUAL_AIRBRAKE:
            if (command->payload_length != 1U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }
            *detail = command->payload[0];
            return (command->payload[0] <= 100U)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_VALUE;

        case ROCKET_CMD_SET_SUBSYSTEM:
            if (command->payload_length != 2U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }
            *detail = ((uint32_t)command->payload[0] << 8) |
                      command->payload[1];
            return ((command->payload[0] <= ROCKET_SUBSYSTEM_MEMORY_LOGGING) &&
                    (command->payload[1] <= 1U))
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_VALUE;

        case ROCKET_CMD_REQUEST_DIAGNOSTICS:
            if (command->payload_length != 1U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }
            *detail = command->payload[0];
            return (command->payload[0] <= ROCKET_DIAG_STORAGE)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_BAD_VALUE;

        default:
            return ROCKET_ACK_UNSUPPORTED;
    }
}

/**
 * @brief Execute bridge-local commands or notify main.c through the callback.
 */
static uint8_t RadioBridge_DispatchCommand(
    const RocketCommandPayload *command,
    uint32_t now_ms,
    uint32_t *detail)
{
    uint8_t result;

    if ((command == NULL) || (detail == NULL))
    {
        return ROCKET_ACK_EXECUTION_ERROR;
    }

    switch (command->command)
    {
        case ROCKET_CMD_NOP:
            *detail = 0U;
            return ROCKET_ACK_OK;

        case ROCKET_CMD_REQUEST_SNAPSHOT:
            /* If normal data is disabled, retain the request until data is
             * enabled again instead of silently losing it. */
            snapshot_requested = 1U;
            *detail = 0U;
            return ROCKET_ACK_OK;

        case ROCKET_CMD_SET_SUBSYSTEM:
            if (command->payload[0] == ROCKET_SUBSYSTEM_RADIO)
            {
                /* Radio disable suppresses telemetry/events only. This ACK and
                 * all future command ACKs deliberately bypass the data gate. */
                radio_data_enabled = (command->payload[1] != 0U);
                if (radio_data_enabled)
                {
                    snapshot_requested = 1U;
                }

                *detail = ((uint32_t)ROCKET_SUBSYSTEM_RADIO << 8) |
                          command->payload[1];
                return ROCKET_ACK_OK;
            }
            break;

        default:
            break;
    }

    /* main.c owns behavior, mode, flight-computer, and airbrake policy. */
    result = RadioBridge_ExecuteCommand(command->command,
                                        command->payload,
                                        command->payload_length,
                                        now_ms,
                                        detail);

    if ((result == ROCKET_ACK_OK) &&
        (command->command == ROCKET_CMD_SET_SUBSYSTEM) &&
        (command->payload[0] == ROCKET_SUBSYSTEM_FLIGHT_COMPUTER))
    {
        /* Mirror main.c's flight-computer state only after successful command
         * execution so failed commands cannot accidentally suppress telemetry. */
        flight_computer_enabled = (command->payload[1] != 0U);
        if (flight_computer_enabled)
        {
            snapshot_requested = 1U;
        }
    }

    return result;
}

/**
 * @brief Decode, validate, execute, and acknowledge one command packet.
 */
static void RadioBridge_ProcessCommand(const uint8_t *packet,
                                       uint8_t length,
                                       uint32_t now_ms)
{
    RocketPacketHeader header;
    RocketCommandPayload command;
    uint8_t result;
    uint32_t detail = 0U;

    if (!RadioBridge_ExtractCommand(packet,
                                    length,
                                    &header,
                                    &command,
                                    &result))
    {
        return;
    }

    if (result == ROCKET_ACK_OK)
    {
        result = RadioBridge_ValidateCommand(&command, &detail);
    }

    if ((result == ROCKET_ACK_OK) &&
        (command.command == ROCKET_CMD_PING))
    {
        RadioBridge_SendPingAcks(header.sequence, now_ms);
        return;
    }

    if ((result == ROCKET_ACK_OK) &&
        (command.command == ROCKET_CMD_REQUEST_DIAGNOSTICS))
    {
        RadioBridge_SendDiagnosticAcks(header.sequence,
                                       &command,
                                       now_ms);
        return;
    }

    /*
     * MOTOR_STEPS is the only normal command whose success ACK is deferred until
     * physical completion.
     */
    if ((result == ROCKET_ACK_OK) &&
        (command.command == ROCKET_CMD_MOTOR_STEPS))
    {
        /*
         * Do not accept a second movement while the first command still owns the
         * deferred ACK slot.
         */
        if (pending_motor_ack != 0U)
        {
            RadioBridge_SendAck(header.sequence,
                                command.command,
                                ROCKET_ACK_BUSY,
                                ROCKET_ACK_DETAIL_NONE,
                                0U,
                                1U,
                                0U,
                                now_ms);
            return;
        }

        result = RadioBridge_DispatchCommand(&command, now_ms, &detail);

        if (result == ROCKET_ACK_OK)
        {
            /*
             * Retain the original ground-command sequence. No success ACK is sent
             * here. The radio task sends it after physical completion.
             */
            pending_motor_ack =
                0x80000000UL | (uint32_t)header.sequence;
            return;
        }

        /*
         * A command that could not start receives an immediate rejection/error.
         * Only an accepted movement waits for completion.
         */
        RadioBridge_SendAck(header.sequence,
                            command.command,
                            result,
                            ROCKET_ACK_DETAIL_NONE,
                            0U,
                            1U,
                            detail,
                            now_ms);
        return;
    }

    if (result == ROCKET_ACK_OK)
    {
        result = RadioBridge_DispatchCommand(&command, now_ms, &detail);
    }

    RadioBridge_SendAck(header.sequence,
                        command.command,
                        result,
                        ROCKET_ACK_DETAIL_NONE,
                        0U,
                        1U,
                        detail,
                        now_ms);
}

/* ========================================================================== */
/* Outgoing packet construction                                               */
/* ========================================================================== */

/** Transmit a complete protocol packet and update saturating link counters. */
static HAL_StatusTypeDef RadioBridge_Transmit(const uint8_t *packet,
                                               uint8_t length)
{
    const HAL_StatusTypeDef status =
        SX1280_Transmit(packet, length, RADIO_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        RadioBridge_IncrementSaturating(&tx_packets_since_ping);
    }
    else
    {
        RadioBridge_IncrementSaturating(&tx_errors_since_ping);
    }

    return status;
}

/**
 * Build the normal 26-byte telemetry payload.
 *
 * Floating-point values are scaled into fixed-point protocol fields before
 * saturation. The packet layout remains defined by rocket_protocol.h.
 */
static HAL_StatusTypeDef RadioBridge_SendTelemetry(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    uint8_t message)
{
    uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 26U];
    size_t index = RocketProtocol_EncodeHeader(packet,
                                               sizeof(packet),
                                               ROCKET_PKT_TELEMETRY,
                                               tx_sequence++,
                                               now_ms);
    const uint16_t flags = RadioBridge_MakeFlags(telemetry);
    const uint16_t failed_reads =
        (telemetry->failed_reads > 65535U)
            ? 65535U
            : (uint16_t)telemetry->failed_reads;

    RocketProtocol_WriteU16(packet + index, flags);
    index += 2U;

    packet[index++] = RadioBridge_MakeState(telemetry);
    packet[index++] = RadioBridge_MapStatus((uint32_t)telemetry->status);

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->ekf_altitude_m * 10.0f));
    index += 2U;

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->ekf_velocity_m_s * 100.0f));
    index += 2U;

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->ekf_acceleration_m_s2 * 100.0f));
    index += 2U;

    RocketProtocol_WriteU16(
        packet + index,
        RadioBridge_ClampU16(telemetry->predicted_apogee_m * 10.0f));
    index += 2U;

    RocketProtocol_WriteU16(
        packet + index,
        RadioBridge_ClampU16(telemetry->target_apogee_m * 10.0f));
    index += 2U;

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->fusion_roll_deg * 10.0f));
    index += 2U;

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->fusion_pitch_deg * 10.0f));
    index += 2U;

    RocketProtocol_WriteI16(
        packet + index,
        RadioBridge_ClampI16(telemetry->fusion_yaw_deg * 10.0f));
    index += 2U;

    packet[index++] = telemetry->controller_requested_percent;
    packet[index++] = RadioBridge_MakeSensorHealth(telemetry);

    RocketProtocol_WriteU16(packet + index, failed_reads);
    index += 2U;

    packet[index++] = message;
    packet[index++] = 0U; /* Reserved protocol byte. */

    return RadioBridge_Transmit(packet, (uint8_t)index);
}

/** Build and transmit a state-transition event packet. */
static HAL_StatusTypeDef RadioBridge_SendEvent(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    uint16_t changed_flags,
    uint8_t old_state,
    uint8_t message)
{
    uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 10U];
    size_t index = RocketProtocol_EncodeHeader(packet,
                                               sizeof(packet),
                                               ROCKET_PKT_EVENT,
                                               tx_sequence++,
                                               now_ms);

    RocketProtocol_WriteU16(packet + index, changed_flags);
    index += 2U;

    RocketProtocol_WriteU16(packet + index,
                            RadioBridge_MakeFlags(telemetry));
    index += 2U;

    packet[index++] = old_state;
    packet[index++] = RadioBridge_MakeState(telemetry);
    packet[index++] = RadioBridge_MapStatus((uint32_t)telemetry->status);
    packet[index++] = message;

    RocketProtocol_WriteU16(packet + index,
                            telemetry->controller_requested_percent);
    index += 2U;

    return RadioBridge_Transmit(packet, (uint8_t)index);
}

/** Build and transmit the current 11-byte acknowledgement payload. */
static void RadioBridge_SendAck(uint16_t command_sequence,
                                uint8_t command,
                                uint8_t result,
                                uint8_t detail_type,
                                uint8_t part_index,
                                uint8_t part_count,
                                uint32_t detail,
                                uint32_t now_ms)
{
    uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 11U];
    size_t index = RocketProtocol_EncodeHeader(packet,
                                               sizeof(packet),
                                               ROCKET_PKT_ACK,
                                               tx_sequence++,
                                               now_ms);

    RocketProtocol_WriteU16(packet + index, command_sequence);
    index += 2U;

    packet[index++] = command;
    packet[index++] = result;
    packet[index++] = detail_type;
    packet[index++] = part_index;
    packet[index++] = part_count;

    RocketProtocol_WriteU32(packet + index, detail);
    index += 4U;

    (void)RadioBridge_Transmit(packet, (uint8_t)index);
}

/** Send a complete ordered set of ACK parts for one command sequence. */
static void RadioBridge_SendAckParts(uint16_t command_sequence,
                                     uint8_t command,
                                     const RadioBridgeAckPart *parts,
                                     uint8_t part_count,
                                     uint32_t now_ms)
{
    uint8_t part_index;

    if ((parts == NULL) || (part_count == 0U))
    {
        return;
    }

    for (part_index = 0U; part_index < part_count; ++part_index)
    {
        RadioBridge_SendAck(command_sequence,
                            command,
                            parts[part_index].result,
                            parts[part_index].detail_type,
                            part_index,
                            part_count,
                            parts[part_index].detail,
                            now_ms);
    }
}

/**
 * Return the three multipart PING ACKs expected by the ground station.
 *
 * Counters are snapshotted and reset before ACK transmission. Therefore these
 * three response packets are counted in the next ping interval, not the one
 * currently being reported.
 */
static void RadioBridge_SendPingAcks(uint16_t command_sequence,
                                     uint32_t now_ms)
{
    const uint8_t tx_packets = tx_packets_since_ping;
    const uint8_t rx_packets = rx_packets_since_ping;
    const uint8_t tx_errors = tx_errors_since_ping;
    const uint8_t rx_errors = rx_errors_since_ping;
    const uint32_t packed_counters =
        RocketProtocol_PackPingCounters(tx_packets,
                                        rx_packets,
                                        tx_errors,
                                        rx_errors);
    const uint32_t packed_signal =
        RocketProtocol_PackPingSignal(last_rssi_dbm_x10,
                                      last_snr_db_x100);
    const RadioBridgeAckPart parts[3] =
    {
        {ROCKET_ACK_OK, ROCKET_ACK_DETAIL_PING_UPTIME, now_ms},
        {ROCKET_ACK_OK, ROCKET_ACK_DETAIL_PING_COUNTERS, packed_counters},
        {ROCKET_ACK_OK, ROCKET_ACK_DETAIL_PING_SIGNAL, packed_signal}
    };

    tx_packets_since_ping = 0U;
    rx_packets_since_ping = 0U;
    tx_errors_since_ping = 0U;
    rx_errors_since_ping = 0U;

    RadioBridge_SendAckParts(command_sequence,
                             ROCKET_CMD_PING,
                             parts,
                             3U,
                             now_ms);
}

/**
 * @brief Stream compressed diagnostic values as multipart ACKs.
 *
 * ACKs are transmitted as they are generated rather than stored in a large
 * temporary array. This minimizes stack usage.
 */
static void RadioBridge_SendDiagnosticAcks(
    uint16_t command_sequence,
    const RocketCommandPayload *command,
    uint32_t now_ms)
{
    /*
     * Number of compressed values returned by each concrete group:
     *
     * RADIO     = 3
     * SENSORS   = 1
     * ESTIMATOR = 3
     * AIRBRAKE  = 6
     * STORAGE   = 1
     *
     * ROCKET_DIAG_ALL therefore sends 14 ACKs.
     */
    static const uint8_t parts_per_group[ROCKET_DIAG_STORAGE + 1U] =
    {
        0U, /* ROCKET_DIAG_ALL is expanded, not returned directly. */
        3U, /* RADIO */
        1U, /* SENSORS */
        3U, /* ESTIMATOR */
        6U, /* AIRBRAKE */
        1U  /* STORAGE */
    };

    RocketCommandPayload diagnostic_command;
    uint8_t first_group;
    uint8_t last_group;
    uint8_t group;
    uint8_t part_index = 0U;
    uint8_t part_count = 0U;

    if ((command == NULL) || (command->payload_length != 1U))
    {
        return;
    }

    if (command->payload[0] == ROCKET_DIAG_ALL)
    {
        first_group = ROCKET_DIAG_RADIO;
        last_group = ROCKET_DIAG_STORAGE;
    }
    else
    {
        first_group = command->payload[0];
        last_group = command->payload[0];
    }

    /*
     * Determine the final total before transmitting the first ACK.
     */
    for (group = first_group; group <= last_group; ++group)
    {
        part_count =
            (uint8_t)(part_count + parts_per_group[group]);
    }

    diagnostic_command = *command;

    /*
     * The original wire request still contains one byte: the requested group.
     * Internally, payload[1] identifies the packed value within that group.
     */
    diagnostic_command.payload_length = 2U;

    for (group = first_group; group <= last_group; ++group)
    {
        uint8_t value_index;

        for (value_index = 0U;
             value_index < parts_per_group[group];
             ++value_index)
        {
            uint8_t result = ROCKET_ACK_OK;
            uint32_t detail = 0U;

            if (group == ROCKET_DIAG_RADIO)
            {
                switch (value_index)
                {
                    case 0U:
                        /*
                         * Four one-byte counters:
                         * [31:24] TX packets
                         * [23:16] RX packets
                         * [15:8]  TX errors
                         * [7:0]   RX errors
                         */
                        detail = RocketProtocol_PackPingCounters(
                            tx_packets_since_ping,
                            rx_packets_since_ping,
                            tx_errors_since_ping,
                            rx_errors_since_ping);
                        break;

                    case 1U:
                        /*
                         * [31:16] RSSI x10
                         * [15:0]  SNR x100
                         */
                        detail = RocketProtocol_PackPingSignal(
                            last_rssi_dbm_x10,
                            last_snr_db_x100);
                        break;

                    case 2U:
                        /*
                         * [31:16] current outgoing sequence
                         * bit 0: normal radio data enabled
                         * bit 1: flight-computer data enabled
                         * bit 2: snapshot pending
                         * bit 3: motor completion ACK pending
                         */
                        detail =
                            ((uint32_t)tx_sequence << 16) |
                            (radio_data_enabled ? 0x01U : 0U) |
                            (flight_computer_enabled ? 0x02U : 0U) |
                            (snapshot_requested ? 0x04U : 0U) |
                            ((pending_motor_ack != 0U) ? 0x08U : 0U);
                        break;

                    default:
                        result = ROCKET_ACK_BAD_VALUE;
                        break;
                }
            }
            else
            {
                diagnostic_command.payload[0] = group;
                diagnostic_command.payload[1] = value_index;

                result = RadioBridge_DispatchCommand(
                    &diagnostic_command,
                    now_ms,
                    &detail);
            }

            RadioBridge_SendAck(command_sequence,
                                ROCKET_CMD_REQUEST_DIAGNOSTICS,
                                result,
                                ROCKET_ACK_DETAIL_DIAGNOSTIC,
                                part_index,
                                part_count,
                                detail,
                                now_ms);

            ++part_index;
        }
    }
}

/* ========================================================================== */
/* General helpers                                                            */
/* ========================================================================== */

/** Increment a one-byte counter without allowing wraparound. */
static void RadioBridge_IncrementSaturating(uint8_t *counter)
{
    if ((counter != NULL) && (*counter < 0xFFU))
    {
        ++(*counter);
    }
}

/** Normal data requires both radio data and flight-computer logic enabled. */
static bool RadioBridge_NormalDataAllowed(void)
{
    return radio_data_enabled && flight_computer_enabled;
}

/* ========================================================================== */
/* Telemetry-to-protocol encoding helpers                                     */
/* ========================================================================== */

/** Round and saturate a float into a signed 16-bit protocol field. */
static int16_t RadioBridge_ClampI16(float value)
{
    if (!isfinite(value))
    {
        return 0;
    }

    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)lroundf(value);
}

/** Round and saturate a nonnegative float into an unsigned 16-bit field. */
static uint16_t RadioBridge_ClampU16(float value)
{
    if (!isfinite(value) || (value <= 0.0f))
    {
        return 0U;
    }

    if (value > 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)lroundf(value);
}

/** HAL_BUSY indicates no fresh sample and is not encoded as a sensor fault. */
static bool RadioBridge_SensorStatusIsFault(HAL_StatusTypeDef status)
{
    return (status != HAL_OK) && (status != HAL_BUSY);
}

/** Convert behavior validity and state booleans into protocol flag bits. */
static uint16_t RadioBridge_MakeFlags(
    const volatile BehaviorTelemetry_t *telemetry)
{
    uint16_t flags = 0U;

    if (telemetry->initialized)
    {
        flags |= ROCKET_FLAG_INITIALIZED;
    }
    if (telemetry->barometer_altitude_valid)
    {
        flags |= ROCKET_FLAG_BARO_VALID;
    }
    if (telemetry->fusion_data_valid)
    {
        flags |= ROCKET_FLAG_FUSION_VALID;
    }
    if (telemetry->ekf_data_valid)
    {
        flags |= ROCKET_FLAG_EKF_VALID;
    }
    if (telemetry->controller_enabled)
    {
        flags |= ROCKET_FLAG_CONTROLLER_ENABLED;
    }
    if (telemetry->controller_active)
    {
        flags |= ROCKET_FLAG_CONTROLLER_ACTIVE;
    }
    if (telemetry->apogee_reached)
    {
        flags |= ROCKET_FLAG_APOGEE_REACHED;
    }
    if (telemetry->mode_changed)
    {
        flags |= ROCKET_FLAG_MODE_CHANGED;
    }
    if (telemetry->barometer_correction_used)
    {
        flags |= ROCKET_FLAG_BARO_CORRECTION_USED;
    }
    if (telemetry->fusion_startup)
    {
        flags |= ROCKET_FLAG_FUSION_STARTUP;
    }
    if (telemetry->fusion_accelerometer_ignored)
    {
        flags |= ROCKET_FLAG_ACCEL_IGNORED;
    }
    if (telemetry->fusion_magnetometer_ignored)
    {
        flags |= ROCKET_FLAG_MAG_IGNORED;
    }
    if (telemetry->full_pipeline_complete)
    {
        flags |= ROCKET_FLAG_PIPELINE_COMPLETE;
    }
    if (telemetry->full_pipeline_pass)
    {
        flags |= ROCKET_FLAG_PIPELINE_PASS;
    }

    if (RadioBridge_SensorStatusIsFault(telemetry->imu_status) ||
        RadioBridge_SensorStatusIsFault(telemetry->mag_status) ||
        RadioBridge_SensorStatusIsFault(telemetry->baro_status))
    {
        flags |= ROCKET_FLAG_SENSOR_FAULT;
    }

    return flags;
}

/** Pack the behavior mode and flight phase into one protocol state byte. */
static uint8_t RadioBridge_MakeState(
    const volatile BehaviorTelemetry_t *telemetry)
{
    return (uint8_t)((((uint8_t)telemetry->mode & 0x0FU) << 4) |
                     ((uint8_t)telemetry->flight_phase & 0x0FU));
}

/** Pack the three two-bit sensor-health fields into one byte. */
static uint8_t RadioBridge_MakeSensorHealth(
    const volatile BehaviorTelemetry_t *telemetry)
{
    const uint8_t imu_health =
        RadioBridge_SensorStatusIsFault(telemetry->imu_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;
    const uint8_t mag_health =
        RadioBridge_SensorStatusIsFault(telemetry->mag_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;
    const uint8_t baro_health =
        RadioBridge_SensorStatusIsFault(telemetry->baro_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;

    return (uint8_t)(imu_health |
                     (uint8_t)(mag_health << 2) |
                     (uint8_t)(baro_health << 4));
}

/** Map internal behavior statuses to stable over-the-air status values. */
static uint8_t RadioBridge_MapStatus(uint32_t status)
{
    switch (status)
    {
        case 0U:
            return ROCKET_STATUS_OK;

        case 1U:
            return ROCKET_STATUS_BAD_ARGUMENT;

        case 2U:
            return ROCKET_STATUS_INVALID_CONFIG;

        case 3U:
            return ROCKET_STATUS_UNSUPPORTED_MODE;

        case 4U:
            return ROCKET_STATUS_SENSOR_INIT_FAILED;

        default:
            return ROCKET_STATUS_UNKNOWN;
    }
}

/** Select the highest-priority message associated with a detected transition. */
static uint8_t RadioBridge_SelectEventMessage(
    const volatile BehaviorTelemetry_t *telemetry,
    uint16_t changed_flags,
    uint8_t old_state,
    uint8_t new_state,
    uint8_t old_deployment)
{
    const uint8_t old_phase = old_state & 0x0FU;
    const uint8_t new_phase = new_state & 0x0FU;

    if (((changed_flags & ROCKET_FLAG_APOGEE_REACHED) != 0U) &&
        telemetry->apogee_reached)
    {
        return ROCKET_MSG_APOGEE_REACHED;
    }

    if (new_phase != old_phase)
    {
        if (new_phase == RADIO_PHASE_LAUNCH)
        {
            return ROCKET_MSG_LAUNCH_DETECTED;
        }

        if (new_phase == RADIO_PHASE_BURNOUT)
        {
            return ROCKET_MSG_BURNOUT_DETECTED;
        }

        if (new_phase == RADIO_PHASE_LANDED)
        {
            return ROCKET_MSG_LANDING_DETECTED;
        }
    }

    if (telemetry->mode_changed ||
        ((new_state >> 4) != (old_state >> 4)))
    {
        return ROCKET_MSG_MODE_CHANGED;
    }

    if ((old_deployment == 0U) &&
        (telemetry->controller_requested_percent > 0U))
    {
        return ROCKET_MSG_AIRBRAKE_DEPLOYED;
    }

    if ((old_deployment > 0U) &&
        (telemetry->controller_requested_percent == 0U))
    {
        return ROCKET_MSG_AIRBRAKE_RETRACTED;
    }

    if ((changed_flags & ROCKET_FLAG_CONTROLLER_ENABLED) != 0U)
    {
        return telemetry->controller_enabled
            ? ROCKET_MSG_CONTROLLER_ENABLED
            : ROCKET_MSG_CONTROLLER_DISABLED;
    }

    return ROCKET_MSG_NONE;
}
