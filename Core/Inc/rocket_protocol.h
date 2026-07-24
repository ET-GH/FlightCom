#ifndef ROCKET_PROTOCOL_H
#define ROCKET_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROCKET_PROTOCOL_MAGIC        0xA5u
#define ROCKET_PROTOCOL_VERSION      0x03u
#define ROCKET_PROTOCOL_HEADER_SIZE  9u
#define ROCKET_PROTOCOL_MAX_PACKET   64u

/* All multibyte values are little-endian. */
typedef enum {
    ROCKET_PKT_TELEMETRY = 0x01,
    ROCKET_PKT_EVENT     = 0x02,
    ROCKET_PKT_COMMAND   = 0x10,
    ROCKET_PKT_ACK       = 0x11,
    ROCKET_PKT_HEARTBEAT = 0x12
} RocketPacketType;

typedef enum {
    ROCKET_CMD_NOP                 = 0x00,
    ROCKET_CMD_PING                = 0x01,
    ROCKET_CMD_REQUEST_SNAPSHOT    = 0x02,
    ROCKET_CMD_SET_TARGET_APOGEE   = 0x10, /* payload: uint16 decimetres */
    ROCKET_CMD_SET_CONTROLLER      = 0x11, /* payload: uint8 0/1 */
    ROCKET_CMD_SET_MODE            = 0x12, /* payload: uint8 mode, uint16 duration_s */
    ROCKET_CMD_RETURN_STANDARD     = 0x13,
    ROCKET_CMD_MANUAL_AIRBRAKE     = 0x14, /* payload: uint8 percent */
    ROCKET_CMD_SET_SUBSYSTEM       = 0x15, /* payload: uint8 subsystem, uint8 0/1 */
    ROCKET_CMD_MOTOR_STEPS         = 0x16, /* payload: int16 signed steps */
    ROCKET_CMD_REQUEST_DIAGNOSTICS = 0x17, /* payload: uint8 diagnostics group */
    ROCKET_CMD_CLEAR_FAULTS        = 0x18, /* no payload */
    ROCKET_CMD_CLEAR_MEMORY        = 0x19  /* no payload */
} RocketCommandCode;


/* Values used by ROCKET_CMD_SET_SUBSYSTEM. "Flight computer" means its
 * software/arming state; the MCU cannot remove its own electrical power. */
typedef enum {
    ROCKET_SUBSYSTEM_RADIO           = 0x00,
    ROCKET_SUBSYSTEM_FLIGHT_COMPUTER = 0x01,
    ROCKET_SUBSYSTEM_MEMORY_LOGGING  = 0x02
} RocketSubsystemId;

/* Values used by ROCKET_CMD_SET_MODE. */
typedef enum {
    ROCKET_MODE_STANDBY = 0x00,
    ROCKET_MODE_ACTIVE  = 0x01,
    ROCKET_MODE_SAFE    = 0x02,
    ROCKET_MODE_TEST    = 0x03
} RocketModeCode;

/* Values used by ROCKET_CMD_REQUEST_DIAGNOSTICS. */
typedef enum {
    ROCKET_DIAG_ALL       = 0x00,
    ROCKET_DIAG_RADIO     = 0x01,
    ROCKET_DIAG_SENSORS   = 0x02,
    ROCKET_DIAG_ESTIMATOR = 0x03,
    ROCKET_DIAG_AIRBRAKE  = 0x04,
    ROCKET_DIAG_STORAGE   = 0x05
} RocketDiagnosticsGroup;

typedef enum {
    ROCKET_ACK_OK              = 0x00,
    ROCKET_ACK_BAD_LENGTH      = 0x01,
    ROCKET_ACK_BAD_VALUE       = 0x02,
    ROCKET_ACK_UNSUPPORTED     = 0x03,
    ROCKET_ACK_BUSY            = 0x04,
    ROCKET_ACK_EXECUTION_ERROR = 0x05,
    ROCKET_ACK_BAD_CRC         = 0x06
} RocketAckCode;

/* detail_type identifies the meaning of the 32-bit ACK detail field. Ping
 * statistics use three ACKs: uptime, compressed counters, and signal data. */
typedef enum {
    ROCKET_ACK_DETAIL_NONE          = 0x00,
    ROCKET_ACK_DETAIL_PING_UPTIME   = 0x01,
    ROCKET_ACK_DETAIL_PING_COUNTERS = 0x02,
    ROCKET_ACK_DETAIL_PING_SIGNAL   = 0x03,
    ROCKET_ACK_DETAIL_DIAGNOSTIC    = 0x10
} RocketAckDetailType;

typedef enum {
    ROCKET_STATUS_OK                 = 0x00,
    ROCKET_STATUS_SENSOR_INIT_FAILED = 0x01,
    ROCKET_STATUS_INVALID_CONFIG     = 0x02,
    ROCKET_STATUS_BAD_ARGUMENT       = 0x03,
    ROCKET_STATUS_UNSUPPORTED_MODE   = 0x04,
    ROCKET_STATUS_SENSOR_READ_FAILED = 0x05,
    ROCKET_STATUS_BARO_STALE         = 0x06,
    ROCKET_STATUS_ESTIMATOR_INVALID  = 0x07,
    ROCKET_STATUS_CONTROLLER_INVALID = 0x08,
    ROCKET_STATUS_RADIO_ERROR        = 0x09,
    ROCKET_STATUS_UNKNOWN            = 0xFF
} RocketStatusCode;

typedef enum {
    ROCKET_MSG_NONE                  = 0x00,
    ROCKET_MSG_BOOT                  = 0x01,
    ROCKET_MSG_RADIO_READY           = 0x02,
    ROCKET_MSG_CONFIG_APPLIED        = 0x03,
    ROCKET_MSG_MODE_CHANGED          = 0x04,
    ROCKET_MSG_LAUNCH_DETECTED       = 0x05,
    ROCKET_MSG_BURNOUT_DETECTED      = 0x06,
    ROCKET_MSG_APOGEE_REACHED        = 0x07,
    ROCKET_MSG_LANDING_DETECTED      = 0x08,
    ROCKET_MSG_AIRBRAKE_DEPLOYED     = 0x09,
    ROCKET_MSG_AIRBRAKE_RETRACTED    = 0x0A,
    ROCKET_MSG_CONTROLLER_ENABLED    = 0x0B,
    ROCKET_MSG_CONTROLLER_DISABLED   = 0x0C,
    ROCKET_MSG_SNAPSHOT              = 0x0D,
    ROCKET_MSG_HEARTBEAT             = 0x0E
} RocketMessageCode;

/* Telemetry flags. One bit replaces multiple bool fields. */
enum {
    ROCKET_FLAG_INITIALIZED          = 1u << 0,
    ROCKET_FLAG_BARO_VALID           = 1u << 1,
    ROCKET_FLAG_FUSION_VALID         = 1u << 2,
    ROCKET_FLAG_EKF_VALID            = 1u << 3,
    ROCKET_FLAG_CONTROLLER_ENABLED   = 1u << 4,
    ROCKET_FLAG_CONTROLLER_ACTIVE    = 1u << 5,
    ROCKET_FLAG_APOGEE_REACHED       = 1u << 6,
    ROCKET_FLAG_MODE_CHANGED         = 1u << 7,
    ROCKET_FLAG_BARO_CORRECTION_USED = 1u << 8,
    ROCKET_FLAG_FUSION_STARTUP       = 1u << 9,
    ROCKET_FLAG_ACCEL_IGNORED        = 1u << 10,
    ROCKET_FLAG_MAG_IGNORED          = 1u << 11,
    ROCKET_FLAG_PIPELINE_COMPLETE    = 1u << 12,
    ROCKET_FLAG_PIPELINE_PASS        = 1u << 13,
    ROCKET_FLAG_SENSOR_FAULT         = 1u << 14,
    ROCKET_FLAG_RESERVED             = 1u << 15
};

/* Sensor health: two bits per sensor: 0 unknown, 1 OK, 2 stale, 3 fault. */
enum {
    ROCKET_SENSOR_UNKNOWN = 0,
    ROCKET_SENSOR_OK      = 1,
    ROCKET_SENSOR_STALE   = 2,
    ROCKET_SENSOR_FAULT   = 3
};

typedef struct {
    uint8_t type;
    uint16_t sequence;
    uint32_t time_ms;
} RocketPacketHeader;

/* 29-byte payload, 38 bytes total including header. */
typedef struct {
    uint16_t flags;
    uint8_t state;            /* upper nibble: behavior mode, lower nibble: flight phase */
    uint8_t status_code;
    int16_t altitude_dm;      /* 0.1 m */
    int16_t velocity_cms;     /* 0.01 m/s */
    int16_t acceleration_cms2;/* 0.01 m/s^2 */
    uint16_t predicted_apogee_dm;
    uint16_t target_apogee_dm;
    int16_t roll_ddeg;        /* 0.1 degree */
    int16_t pitch_ddeg;
    int16_t yaw_ddeg;
    uint8_t deployment_percent;
    uint8_t sensor_health;
    uint16_t failed_reads;
    uint8_t message_code;
    uint8_t reserved;
} RocketTelemetryPayload;

typedef struct {
    uint16_t changed_flags;
    uint16_t current_flags;
    uint8_t previous_state;
    uint8_t current_state;
    uint8_t status_code;
    uint8_t message_code;
    uint16_t detail;
} RocketEventPayload;

typedef struct {
    uint8_t command;
    uint8_t payload_length;
    uint8_t payload[8];
} RocketCommandPayload;

/* 11-byte wire payload. Normal ACKs use part_index=0 and part_count=1.
 * PING uses multiple ACKs sharing the same command_sequence. */
typedef struct {
    uint16_t command_sequence;
    uint8_t command;
    uint8_t result;
    uint8_t detail_type;
    uint8_t part_index;
    uint8_t part_count;
    uint32_t detail;
} RocketAckPayload;

static inline void RocketProtocol_WriteU16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void RocketProtocol_WriteI16(uint8_t *p, int16_t v) {
    RocketProtocol_WriteU16(p, (uint16_t)v);
}
static inline void RocketProtocol_WriteU32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t RocketProtocol_ReadU16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t RocketProtocol_ReadI16(const uint8_t *p) {
    return (int16_t)RocketProtocol_ReadU16(p);
}
static inline uint32_t RocketProtocol_ReadU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Compressed ping counters are counts since the previous ping. Each field
 * saturates at 255. Signal values remain signed fixed-point quantities. */
static inline uint32_t RocketProtocol_PackPingCounters(uint8_t tx_packets,
                                                        uint8_t rx_packets,
                                                        uint8_t tx_errors,
                                                        uint8_t rx_errors) {
    return ((uint32_t)tx_packets << 24) |
           ((uint32_t)rx_packets << 16) |
           ((uint32_t)tx_errors << 8) |
           (uint32_t)rx_errors;
}

static inline uint32_t RocketProtocol_PackPingSignal(int16_t rssi_dbm_x10,
                                                      int16_t snr_db_x100) {
    return ((uint32_t)(uint16_t)rssi_dbm_x10 << 16) |
           (uint16_t)snr_db_x100;
}

static inline uint16_t RocketProtocol_Crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFu;
    size_t i;
    for (i = 0; i < length; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Header = magic, version, type, sequence, time_ms. */
static inline size_t RocketProtocol_EncodeHeader(uint8_t *out, size_t capacity,
                                                  uint8_t type, uint16_t sequence,
                                                  uint32_t time_ms) {
    if (out == NULL || capacity < ROCKET_PROTOCOL_HEADER_SIZE) return 0;
    out[0] = ROCKET_PROTOCOL_MAGIC;
    out[1] = ROCKET_PROTOCOL_VERSION;
    out[2] = type;
    RocketProtocol_WriteU16(out + 3, sequence);
    RocketProtocol_WriteU32(out + 5, time_ms);
    return ROCKET_PROTOCOL_HEADER_SIZE;
}

static inline int RocketProtocol_DecodeHeader(const uint8_t *data, size_t length,
                                               RocketPacketHeader *header) {
    if (!data || !header || length < ROCKET_PROTOCOL_HEADER_SIZE) return 0;
    if (data[0] != ROCKET_PROTOCOL_MAGIC || data[1] != ROCKET_PROTOCOL_VERSION) return 0;
    header->type = data[2];
    header->sequence = RocketProtocol_ReadU16(data + 3);
    header->time_ms = RocketProtocol_ReadU32(data + 5);
    return 1;
}

static inline const char *RocketProtocol_StatusText(uint8_t code) {
    switch (code) {
        case ROCKET_STATUS_OK: return "OK";
        case ROCKET_STATUS_SENSOR_INIT_FAILED: return "SENSOR_INIT_FAILED";
        case ROCKET_STATUS_INVALID_CONFIG: return "INVALID_CONFIG";
        case ROCKET_STATUS_BAD_ARGUMENT: return "BAD_ARGUMENT";
        case ROCKET_STATUS_UNSUPPORTED_MODE: return "UNSUPPORTED_MODE";
        case ROCKET_STATUS_SENSOR_READ_FAILED: return "SENSOR_READ_FAILED";
        case ROCKET_STATUS_BARO_STALE: return "BARO_STALE";
        case ROCKET_STATUS_ESTIMATOR_INVALID: return "ESTIMATOR_INVALID";
        case ROCKET_STATUS_CONTROLLER_INVALID: return "CONTROLLER_INVALID";
        case ROCKET_STATUS_RADIO_ERROR: return "RADIO_ERROR";
        default: return "UNKNOWN_STATUS";
    }
}

static inline const char *RocketProtocol_MessageText(uint8_t code) {
    switch (code) {
        case ROCKET_MSG_NONE: return "NONE";
        case ROCKET_MSG_BOOT: return "BOOT";
        case ROCKET_MSG_RADIO_READY: return "RADIO_READY";
        case ROCKET_MSG_CONFIG_APPLIED: return "CONFIG_APPLIED";
        case ROCKET_MSG_MODE_CHANGED: return "MODE_CHANGED";
        case ROCKET_MSG_LAUNCH_DETECTED: return "LAUNCH_DETECTED";
        case ROCKET_MSG_BURNOUT_DETECTED: return "BURNOUT_DETECTED";
        case ROCKET_MSG_APOGEE_REACHED: return "APOGEE_REACHED";
        case ROCKET_MSG_LANDING_DETECTED: return "LANDING_DETECTED";
        case ROCKET_MSG_AIRBRAKE_DEPLOYED: return "AIRBRAKE_DEPLOYED";
        case ROCKET_MSG_AIRBRAKE_RETRACTED: return "AIRBRAKE_RETRACTED";
        case ROCKET_MSG_CONTROLLER_ENABLED: return "CONTROLLER_ENABLED";
        case ROCKET_MSG_CONTROLLER_DISABLED: return "CONTROLLER_DISABLED";
        case ROCKET_MSG_SNAPSHOT: return "SNAPSHOT";
        case ROCKET_MSG_HEARTBEAT: return "HEARTBEAT";
        default: return "UNKNOWN_MESSAGE";
    }
}

static inline const char *RocketProtocol_CommandText(uint8_t code) {
    switch (code) {
        case ROCKET_CMD_NOP: return "NOP";
        case ROCKET_CMD_PING: return "PING";
        case ROCKET_CMD_REQUEST_SNAPSHOT: return "REQUEST_SNAPSHOT";
        case ROCKET_CMD_SET_TARGET_APOGEE: return "SET_TARGET_APOGEE";
        case ROCKET_CMD_SET_CONTROLLER: return "SET_CONTROLLER";
        case ROCKET_CMD_SET_MODE: return "SET_MODE";
        case ROCKET_CMD_RETURN_STANDARD: return "RETURN_STANDARD";
        case ROCKET_CMD_MANUAL_AIRBRAKE: return "MANUAL_AIRBRAKE";
        case ROCKET_CMD_SET_SUBSYSTEM: return "SET_SUBSYSTEM";
        case ROCKET_CMD_MOTOR_STEPS: return "MOTOR_STEPS";
        case ROCKET_CMD_REQUEST_DIAGNOSTICS: return "REQUEST_DIAGNOSTICS";
        case ROCKET_CMD_CLEAR_FAULTS: return "CLEAR_FAULTS";
        case ROCKET_CMD_CLEAR_MEMORY: return "CLEAR_MEMORY";
        default: return "UNKNOWN_COMMAND";
    }
}

#ifdef __cplusplus
}
#endif
#endif
