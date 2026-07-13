/**
 * @file radio_bridge.c
 * @brief SX1280 telemetry, event reporting, and ground-command bridge.
 *
 * The radio bridge converts BehaviorTelemetry_t into the stable wire format
 * defined by rocket_protocol.h. It also receives command packets, dispatches
 * them to the behavior or airbrake modules, and returns acknowledgements.
 *
 * Normal operation consists of three independent activities:
 *   1. Receive and process pending command packets.
 *   2. Repeat event packets when monitored state changes.
 *   3. Transmit periodic or explicitly requested telemetry snapshots.
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
#define TELEMETRY_PERIOD_MS       100U   /* 10 Hz normal telemetry stream. */
#define HEARTBEAT_PERIOD_MS       1000U
#define EVENT_REPEAT_COUNT        3U

/* Flight-phase values stored in the low nibble of the compact state byte. */
#define RADIO_PHASE_LAUNCH        1U
#define RADIO_PHASE_BURNOUT       2U
#define RADIO_PHASE_LANDED        4U

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

/** Nonzero when the next task iteration must send an immediate snapshot. */
static uint8_t snapshot_requested;

/* -------------------------------------------------------------------------- */
/* Private function declarations                                              */
/* -------------------------------------------------------------------------- */

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
                                uint16_t detail,
                                uint32_t now_ms);
static void RadioBridge_ProcessCommand(const uint8_t *packet,
                                       uint8_t length,
                                       uint32_t now_ms);

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

    tx_sequence = 0U;
    last_telemetry_ms = 0U;
    last_heartbeat_ms = 0U;

    previous_flags = 0U;
    previous_state = 0xFFU;
    previous_status = 0xFFU;
    previous_deployment = 0U;

    snapshot_requested = 1U;

    return status;
}

/** @brief Request an immediate telemetry snapshot on the next task iteration. */
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
 * @param telemetry Current behavior telemetry. A NULL pointer is ignored.
 * @param now_ms Current system timestamp in milliseconds.
 */
void RadioBridge_Task(const volatile BehaviorTelemetry_t *telemetry,
                      uint32_t now_ms)
{
    uint8_t receive_buffer[ROCKET_PROTOCOL_MAX_PACKET];
    uint8_t receive_length = 0U;
    uint16_t flags;
    uint16_t changed_flags;
    uint8_t state;
    uint8_t status;
    uint8_t message;

    if (telemetry == NULL)
    {
        return;
    }

    /* DIO1 is normally interrupt-driven, but polling the pin also catches an
     * event that occurred while interrupts were briefly unavailable. */
    if (dio1_seen ||
        (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) ==
         GPIO_PIN_SET))
    {
        dio1_seen = 0U;

        if (SX1280_ReadPacketIfAvailable(receive_buffer,
                                         &receive_length) == HAL_OK)
        {
            RadioBridge_ProcessCommand(receive_buffer,
                                       receive_length,
                                       now_ms);
        }
    }

    flags = RadioBridge_MakeFlags(telemetry);
    state = RadioBridge_MakeState(telemetry);
    status = RadioBridge_MapStatus((uint32_t)telemetry->status);
    changed_flags = (uint16_t)(flags ^ previous_flags);

    message = RadioBridge_SelectEventMessage(telemetry,
                                             changed_flags,
                                             previous_state,
                                             state,
                                             previous_deployment);

    /* Events are sent redundantly because they represent transitions that may
     * occur between normal telemetry snapshots. */
    if ((changed_flags != 0U) ||
        (state != previous_state) ||
        (status != previous_status) ||
        (telemetry->controller_requested_percent != previous_deployment))
    {
        uint8_t repeat;

        for (repeat = 0U; repeat < EVENT_REPEAT_COUNT; ++repeat)
        {
            (void)RadioBridge_SendEvent(telemetry,
                                        now_ms,
                                        changed_flags,
                                        previous_state,
                                        message);
        }

        previous_flags = flags;
        previous_state = state;
        previous_status = status;
        previous_deployment = telemetry->controller_requested_percent;
    }

    if (snapshot_requested ||
        ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS))
    {
        snapshot_requested = 0U;
        last_telemetry_ms = now_ms;

        (void)RadioBridge_SendTelemetry(telemetry,
                                        now_ms,
                                        ROCKET_MSG_NONE);
    }

    if ((uint32_t)(now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
    {
        last_heartbeat_ms = now_ms;

        /* The periodic binary telemetry stream doubles as the heartbeat, so no
         * additional text or heartbeat-only packet is transmitted here. */
    }
}

/* ========================================================================== */
/* Incoming command processing                                                */
/* ========================================================================== */

/**
 * Decode and execute one command packet, then return a protocol ACK.
 *
 * Malformed headers and packets shorter than the fixed command prefix are
 * silently discarded because a valid sequence number is unavailable. Packets
 * with a valid header but inconsistent payload length receive BAD_LENGTH.
 */
static void RadioBridge_ProcessCommand(const uint8_t *packet,
                                       uint8_t length,
                                       uint32_t now_ms)
{
    RocketPacketHeader header;
    uint8_t command;
    uint8_t payload_length;
    uint8_t result = ROCKET_ACK_OK;
    uint16_t detail = 0U;

    if (!RocketProtocol_DecodeHeader(packet, length, &header) ||
        (header.type != ROCKET_PKT_COMMAND) ||
        (length < 11U))
    {
        return;
    }

    command = packet[9];
    payload_length = packet[10];

    if (((uint16_t)11U + payload_length) > length)
    {
        RadioBridge_SendAck(header.sequence,
                            command,
                            ROCKET_ACK_BAD_LENGTH,
                            length,
                            now_ms);
        return;
    }

    switch (command)
    {
        case ROCKET_CMD_PING:
            break;

        case ROCKET_CMD_REQUEST_SNAPSHOT:
            snapshot_requested = 1U;
            break;

        case ROCKET_CMD_SET_TARGET_APOGEE:
            if (payload_length != 2U)
            {
                result = ROCKET_ACK_BAD_LENGTH;
            }
            else
            {
                const uint16_t target_decimeters =
                    RocketProtocol_ReadU16(packet + 11);

                if (Behavior_SetTargetApogee(
                        (float)target_decimeters / 10.0f,
                        now_ms) != BEHAVIOR_STATUS_OK)
                {
                    result = ROCKET_ACK_BAD_VALUE;
                }

                detail = target_decimeters;
            }
            break;

        case ROCKET_CMD_SET_CONTROLLER:
            if ((payload_length != 1U) || (packet[11] > 1U))
            {
                result = ROCKET_ACK_BAD_VALUE;
            }
            else
            {
                Behavior_SetControllerEnabled(packet[11] != 0U);
                detail = packet[11];
            }
            break;

        case ROCKET_CMD_SET_MODE:
            if (payload_length != 3U)
            {
                result = ROCKET_ACK_BAD_LENGTH;
            }
            else
            {
                const uint8_t mode = packet[11];
                const uint16_t duration_s =
                    RocketProtocol_ReadU16(packet + 12);

                if (Behavior_RequestMode(
                        (BehaviorMode_t)mode,
                        (uint32_t)duration_s * 1000U,
                        now_ms) != BEHAVIOR_STATUS_OK)
                {
                    result = ROCKET_ACK_BAD_VALUE;
                }

                detail = mode;
            }
            break;

        case ROCKET_CMD_RETURN_STANDARD:
            Behavior_ReturnToStandard(now_ms);
            break;

        case ROCKET_CMD_MANUAL_AIRBRAKE:
            if ((payload_length != 1U) || (packet[11] > 100U))
            {
                result = ROCKET_ACK_BAD_VALUE;
            }
            else
            {
                /* Preserve the original behavior: command acceptance is ACKed
                 * after dispatch even if the actuator later reports a fault. */
                Airbrake_SetManualOverride(true);
                Airbrake_SetTargetPercent(packet[11]);
                detail = packet[11];
            }
            break;

        default:
            result = ROCKET_ACK_UNSUPPORTED;
            break;
    }

    RadioBridge_SendAck(header.sequence,
                        command,
                        result,
                        detail,
                        now_ms);
}

/* ========================================================================== */
/* Outgoing packet construction                                               */
/* ========================================================================== */

/** Transmit a complete protocol packet using the bridge-wide timeout. */
static HAL_StatusTypeDef RadioBridge_Transmit(const uint8_t *packet,
                                               uint8_t length)
{
    return SX1280_Transmit(packet, length, RADIO_TX_TIMEOUT_MS);
}

/**
 * Build the normal telemetry packet.
 *
 * Floating-point values are scaled into fixed-point protocol fields before
 * saturation. The packet layout remains defined by rocket_protocol.h.
 */
static HAL_StatusTypeDef RadioBridge_SendTelemetry(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    uint8_t message)
{
    uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 29U];
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

/** Build and transmit a command acknowledgement packet. */
static void RadioBridge_SendAck(uint16_t command_sequence,
                                uint8_t command,
                                uint8_t result,
                                uint16_t detail,
                                uint32_t now_ms)
{
    uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 6U];
    size_t index = RocketProtocol_EncodeHeader(packet,
                                               sizeof(packet),
                                               ROCKET_PKT_ACK,
                                               tx_sequence++,
                                               now_ms);

    RocketProtocol_WriteU16(packet + index, command_sequence);
    index += 2U;

    packet[index++] = command;
    packet[index++] = result;

    RocketProtocol_WriteU16(packet + index, detail);
    index += 2U;

    (void)RadioBridge_Transmit(packet, (uint8_t)index);
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
