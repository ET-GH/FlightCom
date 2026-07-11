#include "radio_bridge.h"
#include "rocket_protocol.h"
#include "sx1280.h"
#include "airbrake.h"
#include <math.h>
#include <string.h>

#define RADIO_TX_TIMEOUT_MS       2000u
#define TELEMETRY_PERIOD_MS       100u   /* 10 Hz normal stream */
#define HEARTBEAT_PERIOD_MS       1000u
#define EVENT_REPEAT_COUNT        3u

static volatile uint8_t dio1_seen;
static uint16_t tx_sequence;
static uint32_t last_telemetry_ms;
static uint32_t last_heartbeat_ms;
static uint16_t previous_flags;
static uint8_t previous_state;
static uint8_t previous_status = 0xFFu;
static uint8_t previous_deployment;
static uint8_t snapshot_requested;

static int16_t clamp_i16(float v) {
    if (!isfinite(v)) return 0;
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}
static uint16_t clamp_u16(float v) {
    if (!isfinite(v) || v <= 0.0f) return 0;
    if (v > 65535.0f) return 65535u;
    return (uint16_t)lroundf(v);
}
static uint16_t sat_u16_u32(uint32_t v) { return v > 65535u ? 65535u : (uint16_t)v; }

static uint16_t make_flags(const volatile BehaviorTelemetry_t *t) {
    uint16_t f = 0;
    if (t->initialized)                    f |= ROCKET_FLAG_INITIALIZED;
    if (t->barometer_altitude_valid)       f |= ROCKET_FLAG_BARO_VALID;
    if (t->fusion_data_valid)              f |= ROCKET_FLAG_FUSION_VALID;
    if (t->ekf_data_valid)                 f |= ROCKET_FLAG_EKF_VALID;
    if (t->controller_enabled)             f |= ROCKET_FLAG_CONTROLLER_ENABLED;
    if (t->controller_active)              f |= ROCKET_FLAG_CONTROLLER_ACTIVE;
    if (t->apogee_reached)                 f |= ROCKET_FLAG_APOGEE_REACHED;
    if (t->mode_changed)                   f |= ROCKET_FLAG_MODE_CHANGED;
    if (t->barometer_correction_used)      f |= ROCKET_FLAG_BARO_CORRECTION_USED;
    if (t->fusion_startup)                 f |= ROCKET_FLAG_FUSION_STARTUP;
    if (t->fusion_accelerometer_ignored)   f |= ROCKET_FLAG_ACCEL_IGNORED;
    if (t->fusion_magnetometer_ignored)    f |= ROCKET_FLAG_MAG_IGNORED;
    if (t->full_pipeline_complete)         f |= ROCKET_FLAG_PIPELINE_COMPLETE;
    if (t->full_pipeline_pass)             f |= ROCKET_FLAG_PIPELINE_PASS;
    if ((t->imu_status != HAL_OK) || (t->mag_status != HAL_OK) || (t->baro_status != HAL_OK))
        f |= ROCKET_FLAG_SENSOR_FAULT;
    return f;
}

static uint8_t make_state(const volatile BehaviorTelemetry_t *t) {
    return (uint8_t)((((uint8_t)t->mode & 0x0Fu) << 4) | ((uint8_t)t->flight_phase & 0x0Fu));
}

static uint8_t health2(HAL_StatusTypeDef s) {
    return (s == HAL_OK) ? ROCKET_SENSOR_OK : ROCKET_SENSOR_FAULT;
}
static uint8_t make_sensor_health(const volatile BehaviorTelemetry_t *t) {
    return (uint8_t)(health2(t->imu_status) |
                    (health2(t->mag_status) << 2) |
                    (health2(t->baro_status) << 4));
}

static uint8_t map_status(uint32_t s) {
    /* BehaviorStatus_t currently uses compact enum values; explicit mapping keeps RF stable. */
    switch (s) {
        case 0: return ROCKET_STATUS_OK;
        case 1: return ROCKET_STATUS_BAD_ARGUMENT;
        case 2: return ROCKET_STATUS_INVALID_CONFIG;
        case 3: return ROCKET_STATUS_UNSUPPORTED_MODE;
        case 4: return ROCKET_STATUS_SENSOR_INIT_FAILED;
        default: return ROCKET_STATUS_UNKNOWN;
    }
}

static uint8_t event_message(const volatile BehaviorTelemetry_t *t, uint16_t changed,
                             uint8_t old_state, uint8_t new_state, uint8_t old_dep) {
    uint8_t old_phase = old_state & 0x0Fu;
    uint8_t new_phase = new_state & 0x0Fu;
    if ((changed & ROCKET_FLAG_APOGEE_REACHED) && t->apogee_reached) return ROCKET_MSG_APOGEE_REACHED;
    if (new_phase != old_phase) {
        if (new_phase == 1u) return ROCKET_MSG_LAUNCH_DETECTED;
        if (new_phase == 2u) return ROCKET_MSG_BURNOUT_DETECTED;
        if (new_phase == 4u) return ROCKET_MSG_LANDING_DETECTED;
    }
    if (t->mode_changed || ((new_state >> 4) != (old_state >> 4))) return ROCKET_MSG_MODE_CHANGED;
    if (old_dep == 0u && t->controller_requested_percent > 0u) return ROCKET_MSG_AIRBRAKE_DEPLOYED;
    if (old_dep > 0u && t->controller_requested_percent == 0u) return ROCKET_MSG_AIRBRAKE_RETRACTED;
    if (changed & ROCKET_FLAG_CONTROLLER_ENABLED)
        return t->controller_enabled ? ROCKET_MSG_CONTROLLER_ENABLED : ROCKET_MSG_CONTROLLER_DISABLED;
    return ROCKET_MSG_NONE;
}

static HAL_StatusTypeDef transmit(const uint8_t *packet, uint8_t length) {
    return SX1280_Transmit(packet, length, RADIO_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef send_telemetry(const volatile BehaviorTelemetry_t *t, uint32_t now_ms,
                                        uint8_t message) {
    uint8_t p[ROCKET_PROTOCOL_HEADER_SIZE + 29u];
    size_t i = RocketProtocol_EncodeHeader(p, sizeof(p), ROCKET_PKT_TELEMETRY, tx_sequence++, now_ms);
    uint16_t flags = make_flags(t);
    RocketProtocol_WriteU16(p + i, flags); i += 2;
    p[i++] = make_state(t);
    p[i++] = map_status((uint32_t)t->status);
    RocketProtocol_WriteI16(p + i, clamp_i16(t->ekf_altitude_m * 10.0f)); i += 2;
    RocketProtocol_WriteI16(p + i, clamp_i16(t->ekf_velocity_m_s * 100.0f)); i += 2;
    RocketProtocol_WriteI16(p + i, clamp_i16(t->ekf_acceleration_m_s2 * 100.0f)); i += 2;
    RocketProtocol_WriteU16(p + i, clamp_u16(t->predicted_apogee_m * 10.0f)); i += 2;
    RocketProtocol_WriteU16(p + i, clamp_u16(t->target_apogee_m * 10.0f)); i += 2;
    RocketProtocol_WriteI16(p + i, clamp_i16(t->fusion_roll_deg * 10.0f)); i += 2;
    RocketProtocol_WriteI16(p + i, clamp_i16(t->fusion_pitch_deg * 10.0f)); i += 2;
    RocketProtocol_WriteI16(p + i, clamp_i16(t->fusion_yaw_deg * 10.0f)); i += 2;
    p[i++] = t->controller_requested_percent;
    p[i++] = make_sensor_health(t);
    RocketProtocol_WriteU16(p + i, sat_u16_u32(t->failed_reads)); i += 2;
    p[i++] = message;
    p[i++] = 0;
    return transmit(p, (uint8_t)i);
}

static HAL_StatusTypeDef send_event(const volatile BehaviorTelemetry_t *t, uint32_t now_ms,
                                    uint16_t changed, uint8_t old_state, uint8_t message) {
    uint8_t p[ROCKET_PROTOCOL_HEADER_SIZE + 10u];
    size_t i = RocketProtocol_EncodeHeader(p, sizeof(p), ROCKET_PKT_EVENT, tx_sequence++, now_ms);
    RocketProtocol_WriteU16(p + i, changed); i += 2;
    RocketProtocol_WriteU16(p + i, make_flags(t)); i += 2;
    p[i++] = old_state;
    p[i++] = make_state(t);
    p[i++] = map_status((uint32_t)t->status);
    p[i++] = message;
    RocketProtocol_WriteU16(p + i, t->controller_requested_percent); i += 2;
    return transmit(p, (uint8_t)i);
}

static void send_ack(uint16_t command_seq, uint8_t command, uint8_t result, uint16_t detail, uint32_t now_ms) {
    uint8_t p[ROCKET_PROTOCOL_HEADER_SIZE + 6u];
    size_t i = RocketProtocol_EncodeHeader(p, sizeof(p), ROCKET_PKT_ACK, tx_sequence++, now_ms);
    RocketProtocol_WriteU16(p + i, command_seq); i += 2;
    p[i++] = command;
    p[i++] = result;
    RocketProtocol_WriteU16(p + i, detail); i += 2;
    (void)transmit(p, (uint8_t)i);
}

static void process_command(const uint8_t *p, uint8_t len, uint32_t now_ms) {
    RocketPacketHeader h;
    uint8_t cmd, n, result = ROCKET_ACK_OK;
    uint16_t detail = 0;
    if (!RocketProtocol_DecodeHeader(p, len, &h) || h.type != ROCKET_PKT_COMMAND || len < 11u) return;
    cmd = p[9]; n = p[10];
    if ((uint16_t)11u + n > len) { send_ack(h.sequence, cmd, ROCKET_ACK_BAD_LENGTH, len, now_ms); return; }
    switch (cmd) {
        case ROCKET_CMD_PING: break;
        case ROCKET_CMD_REQUEST_SNAPSHOT: snapshot_requested = 1u; break;
        case ROCKET_CMD_SET_TARGET_APOGEE:
            if (n != 2u) result = ROCKET_ACK_BAD_LENGTH;
            else {
                uint16_t dm = RocketProtocol_ReadU16(p + 11);
                if (Behavior_SetTargetApogee((float)dm / 10.0f, now_ms) != BEHAVIOR_STATUS_OK)
                    result = ROCKET_ACK_BAD_VALUE;
                detail = dm;
            }
            break;
        case ROCKET_CMD_SET_CONTROLLER:
            if (n != 1u || p[11] > 1u) result = ROCKET_ACK_BAD_VALUE;
            else { Behavior_SetControllerEnabled(p[11] != 0u); detail = p[11]; }
            break;
        case ROCKET_CMD_SET_MODE:
            if (n != 3u) result = ROCKET_ACK_BAD_LENGTH;
            else {
                uint8_t mode = p[11];
                uint16_t duration_s = RocketProtocol_ReadU16(p + 12);
                if (Behavior_RequestMode((BehaviorMode_t)mode, (uint32_t)duration_s * 1000u, now_ms) != BEHAVIOR_STATUS_OK)
                    result = ROCKET_ACK_BAD_VALUE;
                detail = mode;
            }
            break;
        case ROCKET_CMD_RETURN_STANDARD: Behavior_ReturnToStandard(now_ms); break;
        case ROCKET_CMD_MANUAL_AIRBRAKE:
            if (n != 1u || p[11] > 100u) result = ROCKET_ACK_BAD_VALUE;
            else { Airbrake_SetManualOverride(true); Airbrake_SetTargetPercent(p[11]); detail = p[11]; }
            break;
        default: result = ROCKET_ACK_UNSUPPORTED; break;
    }
    send_ack(h.sequence, cmd, result, detail, now_ms);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == LORA_DIO1_Pin) dio1_seen = 1u;
}

HAL_StatusTypeDef RadioBridge_Init(void) {
    HAL_StatusTypeDef s = SX1280_InitLoRa();
    tx_sequence = 0; last_telemetry_ms = 0; last_heartbeat_ms = 0;
    previous_flags = 0; previous_state = 0xFFu; previous_status = 0xFFu;
    previous_deployment = 0; snapshot_requested = 1;
    return s;
}

void RadioBridge_RequestSnapshot(void) { snapshot_requested = 1u; }

void RadioBridge_Task(const volatile BehaviorTelemetry_t *t, uint32_t now_ms) {
    uint8_t rx[ROCKET_PROTOCOL_MAX_PACKET];
    uint8_t rx_len = 0;
    uint16_t flags, changed;
    uint8_t state, status, message;
    if (!t) return;

    if (dio1_seen || HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) == GPIO_PIN_SET) {
        dio1_seen = 0;
        if (SX1280_ReadPacketIfAvailable(rx, &rx_len) == HAL_OK) process_command(rx, rx_len, now_ms);
    }

    flags = make_flags(t); state = make_state(t); status = map_status((uint32_t)t->status);
    changed = (uint16_t)(flags ^ previous_flags);
    message = event_message(t, changed, previous_state, state, previous_deployment);

    if (changed || state != previous_state || status != previous_status ||
        t->controller_requested_percent != previous_deployment) {
        uint8_t r;
        for (r = 0; r < EVENT_REPEAT_COUNT; ++r)
            (void)send_event(t, now_ms, changed, previous_state, message);
        previous_flags = flags; previous_state = state; previous_status = status;
        previous_deployment = t->controller_requested_percent;
    }

    if (snapshot_requested || (uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
        snapshot_requested = 0;
        last_telemetry_ms = now_ms;
        (void)send_telemetry(t, now_ms, ROCKET_MSG_NONE);
    }

    if ((uint32_t)(now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS) {
        last_heartbeat_ms = now_ms;
        /* Telemetry doubles as heartbeat; no redundant text packet. */
    }
}
