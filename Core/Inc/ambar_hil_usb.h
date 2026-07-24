/*
 * AMBAR PORTABLE USB HIL TRANSPORT
 *
 * This two-file module implements AMBAR Rocket Protocol v2 over any
 * nonblocking byte stream.  It deliberately contains no HAL, USBX, RTOS,
 * filesystem, radio, or heap dependency.  The caller supplies four callbacks
 * and calls AmbarHilUsb_Poll() frequently from its cooperative main loop.
 *
 * IMPORTANT INTEGRATION RULES
 * ---------------------------
 * 1. This module does not initialize USB.  Initialize the board's existing USB
 *    device stack exactly once, then bind these callbacks to the already-open
 *    CDC byte stream.  Do not create a second USBX pool, device stack, CDC
 *    class, endpoint, or PCD initialization path.
 * 2. Do not run this framing layer on top of another Rocket Protocol framing
 *    layer.  Bind it to raw CDC bytes, or keep the existing ambar_usb transport.
 * 3. A decoded command is not authorization to move hardware.  The application
 *    remains responsible for build-profile, arming, HOME, estimator, freshness,
 *    inhibit, limit, ESTOP, and actuator safety checks before execution.
 * 4. All callbacks must return immediately.  AMBAR_HIL_USB_IO_ERROR reports a
 *    transport fault; otherwise read/write return a byte count no greater than
 *    the supplied capacity.
 *
 * Minimal binding sketch (names are intentionally board-specific placeholders):
 *
 *   static AmbarHilUsb usb_hil;
 *
 *   static size_t usb_read(void *u, uint8_t *dst, size_t cap) {
 *       return BoardCdc_ReadNonBlocking(u, dst, cap);
 *   }
 *   static size_t usb_write(void *u, const uint8_t *src, size_t len) {
 *       return BoardCdc_WriteNonBlocking(u, src, len);
 *   }
 *   static uint32_t usb_now(void *u) { return BoardMillis(u); }
 *   static bool usb_connected(void *u) { return BoardCdc_IsConfigured(u); }
 *
 *   void HilTransport_Init(void) {
 *       AmbarHilUsbIo io = { usb_read, usb_write, usb_now, usb_connected,
 *                            &existing_cdc_context };
 *       (void)AmbarHilUsb_Init(&usb_hil, &io); // USBX was initialized elsewhere
 *   }
 *   void MainLoop(void) {
 *       AmbarHilUsbMessage message;
 *       AmbarHilUsb_Poll(&usb_hil);
 *       while (AmbarHilUsb_TakeMessage(&usb_hil, &message)) {
 *           Application_ValidateAndHandle(&message);
 *       }
 *   }
 */

#ifndef AMBAR_HIL_USB_H
#define AMBAR_HIL_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rocket Protocol v2 wire limits. */
#define AMBAR_HIL_USB_MAGIC                    0xA5u
#define AMBAR_HIL_USB_PROTOCOL_VERSION         0x02u
#define AMBAR_HIL_USB_HEADER_SIZE              9u
#define AMBAR_HIL_USB_CRC_SIZE                 2u
#define AMBAR_HIL_USB_MAX_PACKET_SIZE          64u
#define AMBAR_HIL_USB_MAX_PAYLOAD_SIZE         53u
#define AMBAR_HIL_USB_MAX_FRAME_SIZE           66u
#define AMBAR_HIL_USB_FRAME_DELIMITER          0x00u
#define AMBAR_HIL_USB_COMMAND_DATA_MAX         8u
#define AMBAR_HIL_USB_CDA_POINT_COUNT          5u
#define AMBAR_HIL_USB_VARIABLE_CONFIG_VERSION  1u
#define AMBAR_HIL_USB_FRACTION_U16_FULL_SCALE  65535u
#define AMBAR_HIL_USB_RECOVER_FULL_MAGIC       0x4C4C5546UL
#define AMBAR_HIL_USB_RECOVER_FULL_DATA_SIZE   4u
#define AMBAR_HIL_USB_RX_QUEUE_DEPTH           8u
#define AMBAR_HIL_USB_TX_QUEUE_DEPTH           8u
#define AMBAR_HIL_USB_IO_ERROR                 ((size_t)-1)

#define AMBAR_HIL_USB_TELEMETRY_PAYLOAD_SIZE   26u
#define AMBAR_HIL_USB_EVENT_PAYLOAD_SIZE       10u
#define AMBAR_HIL_USB_COMMAND_PREFIX_SIZE      2u
#define AMBAR_HIL_USB_ACK_PAYLOAD_SIZE          6u
#define AMBAR_HIL_USB_SIMULATION_PAYLOAD_SIZE  16u
#define AMBAR_HIL_USB_ACTUATOR_PAYLOAD_SIZE    24u
#define AMBAR_HIL_USB_VARIABLE_STATE_SIZE      44u
#define AMBAR_HIL_USB_VARIABLE_CONFIG_SIZE     52u
#define AMBAR_HIL_USB_HEARTBEAT_PAYLOAD_SIZE    8u
#define AMBAR_HIL_USB_LOG_STATUS_PAYLOAD_SIZE  16u

typedef enum
{
    AMBAR_HIL_USB_PACKET_TELEMETRY = 0x01,
    AMBAR_HIL_USB_PACKET_EVENT = 0x02,
    AMBAR_HIL_USB_PACKET_ACTUATOR_STATUS = 0x03,
    AMBAR_HIL_USB_PACKET_VARIABLE_HIL_STATE = 0x04,
    AMBAR_HIL_USB_PACKET_VARIABLE_HIL_CONFIG = 0x05,
    AMBAR_HIL_USB_PACKET_COMMAND = 0x10,
    AMBAR_HIL_USB_PACKET_ACK = 0x11,
    AMBAR_HIL_USB_PACKET_HEARTBEAT = 0x12,
    AMBAR_HIL_USB_PACKET_LOG_STATUS = 0x13,
    AMBAR_HIL_USB_PACKET_SIMULATION = 0x20,
    AMBAR_HIL_USB_PACKET_VARIABLE_HIL_CONFIG_UPLOAD = 0x21
} AmbarHilUsbPacketType;

typedef enum
{
    AMBAR_HIL_USB_COMMAND_NOP = 0x00,
    AMBAR_HIL_USB_COMMAND_PING = 0x01,
    AMBAR_HIL_USB_COMMAND_REQUEST_SNAPSHOT = 0x02,
    AMBAR_HIL_USB_COMMAND_SET_TARGET_APOGEE = 0x10,
    AMBAR_HIL_USB_COMMAND_SET_ARMED = 0x11,
    AMBAR_HIL_USB_COMMAND_SET_MODE = 0x12,
    AMBAR_HIL_USB_COMMAND_RETURN_STANDARD = 0x13,
    AMBAR_HIL_USB_COMMAND_MANUAL_AIRBRAKE = 0x14,
    AMBAR_HIL_USB_COMMAND_ESTOP = 0x15,
    AMBAR_HIL_USB_COMMAND_HOME = 0x16,
    AMBAR_HIL_USB_COMMAND_RETRACT = 0x17,
    AMBAR_HIL_USB_COMMAND_PAD_RESET = 0x18,
    AMBAR_HIL_USB_COMMAND_SAVE_CONFIG = 0x19,
    AMBAR_HIL_USB_COMMAND_BENCH_MOVE_STEPS = 0x1A,
    AMBAR_HIL_USB_COMMAND_SIM_START = 0x20,
    AMBAR_HIL_USB_COMMAND_SIM_STOP = 0x21,
    AMBAR_HIL_USB_COMMAND_HIL_SET_OVERRIDE = 0x22,
    AMBAR_HIL_USB_COMMAND_VARIABLE_HIL_GET_CONFIG = 0x23,
    AMBAR_HIL_USB_COMMAND_VARIABLE_HIL_CONFIG_UPLOAD = 0x24,
    AMBAR_HIL_USB_COMMAND_RECOVER_KNOWN_FULL_RETRACT = 0x25,
    AMBAR_HIL_USB_COMMAND_START_LOG = 0x30,
    AMBAR_HIL_USB_COMMAND_STOP_LOG = 0x31,
    AMBAR_HIL_USB_COMMAND_ERASE_LOG = 0x32,
    AMBAR_HIL_USB_COMMAND_EXPORT_LOG = 0x33,
    AMBAR_HIL_USB_COMMAND_CANCEL_EXPORT = 0x34
} AmbarHilUsbCommandCode;

/* Source-compatible vocabulary alias; both names are wire value 0x11. */
#define AMBAR_HIL_USB_COMMAND_SET_CONTROLLER \
    AMBAR_HIL_USB_COMMAND_SET_ARMED

typedef enum
{
    AMBAR_HIL_USB_OVERRIDE_OFF = 0,
    AMBAR_HIL_USB_OVERRIDE_FORCE_FULL = 1,
    AMBAR_HIL_USB_OVERRIDE_FORCE_HOME = 2
} AmbarHilUsbOverrideMode;

typedef enum
{
    AMBAR_HIL_USB_ACK_OK = 0x00,
    AMBAR_HIL_USB_ACK_BAD_LENGTH = 0x01,
    AMBAR_HIL_USB_ACK_BAD_VALUE = 0x02,
    AMBAR_HIL_USB_ACK_UNSUPPORTED = 0x03,
    AMBAR_HIL_USB_ACK_BUSY = 0x04,
    AMBAR_HIL_USB_ACK_EXECUTION_ERROR = 0x05,
    AMBAR_HIL_USB_ACK_BAD_CRC = 0x06
} AmbarHilUsbAckCode;

/*
 * States carried by AMBAR_HIL_USB_PACKET_LOG_STATUS.
 *
 * STARTED supplies the full 32-bit record total before archive packets begin.
 * COMPLETE, CANCELLED, and ERROR terminate the transfer unambiguously.
 */
typedef enum
{
    AMBAR_HIL_USB_LOG_TRANSFER_STARTED = 0x01,
    AMBAR_HIL_USB_LOG_TRANSFER_PROGRESS = 0x02,
    AMBAR_HIL_USB_LOG_TRANSFER_COMPLETE = 0x03,
    AMBAR_HIL_USB_LOG_TRANSFER_CANCELLED = 0x04,
    AMBAR_HIL_USB_LOG_TRANSFER_ERROR = 0x05
} AmbarHilUsbLogTransferState;

typedef enum
{
    AMBAR_HIL_USB_STATUS_OK = 0x00,
    AMBAR_HIL_USB_STATUS_SENSOR_INIT_FAILED = 0x01,
    AMBAR_HIL_USB_STATUS_INVALID_CONFIG = 0x02,
    AMBAR_HIL_USB_STATUS_BAD_ARGUMENT = 0x03,
    AMBAR_HIL_USB_STATUS_UNSUPPORTED_MODE = 0x04,
    AMBAR_HIL_USB_STATUS_SENSOR_READ_FAILED = 0x05,
    AMBAR_HIL_USB_STATUS_BARO_STALE = 0x06,
    AMBAR_HIL_USB_STATUS_ESTIMATOR_INVALID = 0x07,
    AMBAR_HIL_USB_STATUS_CONTROLLER_INVALID = 0x08,
    AMBAR_HIL_USB_STATUS_RADIO_ERROR = 0x09,
    AMBAR_HIL_USB_STATUS_USB_ERROR = 0x0A,
    AMBAR_HIL_USB_STATUS_SIMULATION_STALE = 0x0B,
    AMBAR_HIL_USB_STATUS_UNKNOWN = 0xFF
} AmbarHilUsbStatusCode;

typedef enum
{
    AMBAR_HIL_USB_MESSAGE_NONE = 0x00,
    AMBAR_HIL_USB_MESSAGE_BOOT = 0x01,
    AMBAR_HIL_USB_MESSAGE_RADIO_READY = 0x02,
    AMBAR_HIL_USB_MESSAGE_CONFIG_APPLIED = 0x03,
    AMBAR_HIL_USB_MESSAGE_MODE_CHANGED = 0x04,
    AMBAR_HIL_USB_MESSAGE_LAUNCH_DETECTED = 0x05,
    AMBAR_HIL_USB_MESSAGE_BURNOUT_DETECTED = 0x06,
    AMBAR_HIL_USB_MESSAGE_APOGEE_REACHED = 0x07,
    AMBAR_HIL_USB_MESSAGE_LANDING_DETECTED = 0x08,
    AMBAR_HIL_USB_MESSAGE_AIRBRAKE_DEPLOYED = 0x09,
    AMBAR_HIL_USB_MESSAGE_AIRBRAKE_RETRACTED = 0x0A,
    AMBAR_HIL_USB_MESSAGE_CONTROLLER_ENABLED = 0x0B,
    AMBAR_HIL_USB_MESSAGE_CONTROLLER_DISABLED = 0x0C,
    AMBAR_HIL_USB_MESSAGE_SNAPSHOT = 0x0D,
    AMBAR_HIL_USB_MESSAGE_HEARTBEAT = 0x0E,
    AMBAR_HIL_USB_MESSAGE_SIMULATION_STARTED = 0x0F,
    AMBAR_HIL_USB_MESSAGE_SIMULATION_STOPPED = 0x10,
    AMBAR_HIL_USB_MESSAGE_SIMULATION_STALE = 0x11
} AmbarHilUsbMessageCode;

enum
{
    AMBAR_HIL_USB_FLAG_INITIALIZED = 1u << 0,
    AMBAR_HIL_USB_FLAG_BARO_VALID = 1u << 1,
    AMBAR_HIL_USB_FLAG_FUSION_VALID = 1u << 2,
    AMBAR_HIL_USB_FLAG_EKF_VALID = 1u << 3,
    AMBAR_HIL_USB_FLAG_ARMED = 1u << 4,
    AMBAR_HIL_USB_FLAG_CONTROLLER_ACTIVE = 1u << 5,
    AMBAR_HIL_USB_FLAG_APOGEE_REACHED = 1u << 6,
    AMBAR_HIL_USB_FLAG_MODE_CHANGED = 1u << 7,
    AMBAR_HIL_USB_FLAG_BARO_CORRECTION_USED = 1u << 8,
    AMBAR_HIL_USB_FLAG_FUSION_STARTUP = 1u << 9,
    AMBAR_HIL_USB_FLAG_ACCEL_IGNORED = 1u << 10,
    AMBAR_HIL_USB_FLAG_MAG_IGNORED = 1u << 11,
    AMBAR_HIL_USB_FLAG_PIPELINE_COMPLETE = 1u << 12,
    AMBAR_HIL_USB_FLAG_PIPELINE_PASS = 1u << 13,
    AMBAR_HIL_USB_FLAG_SENSOR_FAULT = 1u << 14,
    AMBAR_HIL_USB_FLAG_SIMULATION_ACTIVE = 1u << 15
};

enum
{
    AMBAR_HIL_USB_SIM_ALTITUDE_VALID = 1u << 0,
    AMBAR_HIL_USB_SIM_ACCELERATION_VALID = 1u << 1,
    AMBAR_HIL_USB_SIM_VELOCITY_VALID = 1u << 2,
    AMBAR_HIL_USB_SIM_END_OF_STREAM = 1u << 3
};

enum
{
    AMBAR_HIL_USB_ACTUATOR_BUILD_ENABLED = 1u << 0,
    AMBAR_HIL_USB_ACTUATOR_BENCH_ENABLED = 1u << 1,
    AMBAR_HIL_USB_ACTUATOR_HOMED = 1u << 2,
    AMBAR_HIL_USB_ACTUATOR_DRIVER_OK = 1u << 3,
    AMBAR_HIL_USB_ACTUATOR_DRIVER_ENABLED = 1u << 4,
    AMBAR_HIL_USB_ACTUATOR_ESTOP = 1u << 5,
    AMBAR_HIL_USB_ACTUATOR_CONFIG_VALID = 1u << 6,
    AMBAR_HIL_USB_ACTUATOR_MANUAL_PENDING = 1u << 7
};

enum
{
    AMBAR_HIL_USB_ACTUATOR_STATUS_SOFTWARE_HOME = 1u << 0,
    AMBAR_HIL_USB_ACTUATOR_STATUS_SOFTWARE_FULL = 1u << 1,
    AMBAR_HIL_USB_ACTUATOR_STATUS_GEOMETRY_VALID = 1u << 2,
    AMBAR_HIL_USB_ACTUATOR_STATUS_OVERRIDE_ACTIVE = 1u << 3,
    AMBAR_HIL_USB_ACTUATOR_STATUS_OVERRIDE_SHIFT = 4,
    AMBAR_HIL_USB_ACTUATOR_STATUS_OVERRIDE_MASK = 3u << 4,
    AMBAR_HIL_USB_ACTUATOR_STATUS_CONTINUOUS_HIL = 1u << 6,
    AMBAR_HIL_USB_ACTUATOR_STATUS_STROKE_VERIFIED = 1u << 7,
    AMBAR_HIL_USB_ACTUATOR_STATUS_VARIABLE_HIL = 1u << 8
};

enum
{
    AMBAR_HIL_USB_VARIABLE_DRIVER_OK = 1u << 0,
    AMBAR_HIL_USB_VARIABLE_DRIVER_ENABLED = 1u << 1,
    AMBAR_HIL_USB_VARIABLE_CONFIG_VALID = 1u << 2,
    AMBAR_HIL_USB_VARIABLE_SIM_ACTIVE = 1u << 3,
    AMBAR_HIL_USB_VARIABLE_SIM_FRESH = 1u << 4,
    AMBAR_HIL_USB_VARIABLE_ARMED = 1u << 5,
    AMBAR_HIL_USB_VARIABLE_SOFTWARE_HOME = 1u << 6,
    AMBAR_HIL_USB_VARIABLE_TARGET_REACHABLE = 1u << 7
};

/* Heartbeat feature_flags bit assignments reported by the active firmware. */
enum
{
    AMBAR_HIL_USB_FEATURE_RADIO = 1u << 0,
    AMBAR_HIL_USB_FEATURE_RADIO_TELEMETRY = 1u << 1,
    AMBAR_HIL_USB_FEATURE_FLIGHT_LOGGING = 1u << 2,
    AMBAR_HIL_USB_FEATURE_ACTUATOR = 1u << 3,
    AMBAR_HIL_USB_FEATURE_BENCH_COMMANDS = 1u << 4,
    AMBAR_HIL_USB_FEATURE_WATCHDOG = 1u << 5,
    AMBAR_HIL_USB_FEATURE_MAGNETOMETER = 1u << 6,
    AMBAR_HIL_USB_FEATURE_VERBOSE_TEXT = 1u << 7,
    AMBAR_HIL_USB_FEATURE_AUTO_FLIGHT_PHASES = 1u << 8,
    AMBAR_HIL_USB_FEATURE_RADIO_HEARTBEAT = 1u << 9,
    AMBAR_HIL_USB_FEATURE_DIRECT_USB = 1u << 10,
    AMBAR_HIL_USB_FEATURE_SIMULATION_INPUT = 1u << 11,
    AMBAR_HIL_USB_FEATURE_VBUS_SENSE = 1u << 12,
    AMBAR_HIL_USB_FEATURE_USB_FLASH_MAINTENANCE = 1u << 13,
    AMBAR_HIL_USB_FEATURE_CONTINUOUS_HIL = 1u << 14,
    AMBAR_HIL_USB_FEATURE_ACTUATOR_DIRECTION_INVERTED = 1u << 15,
    AMBAR_HIL_USB_FEATURE_VARIABLE_HIL = 1u << 16
};

typedef enum
{
    AMBAR_HIL_USB_SENSOR_UNKNOWN = 0,
    AMBAR_HIL_USB_SENSOR_OK = 1,
    AMBAR_HIL_USB_SENSOR_STALE = 2,
    AMBAR_HIL_USB_SENSOR_FAULT = 3
} AmbarHilUsbSensorHealth;

typedef enum
{
    AMBAR_HIL_USB_FEEDBACK_UNKNOWN = 0,
    /* TMC5240 ramp-generator XACTUAL, not encoder/mechanical feedback. */
    AMBAR_HIL_USB_FEEDBACK_TMC5240_XACTUAL = 1
} AmbarHilUsbFeedbackSource;

typedef struct
{
    uint16_t flags;
    uint8_t state;
    uint8_t status_code;
    int16_t altitude_dm;
    int16_t velocity_cms;
    int16_t acceleration_cms2;
    uint16_t predicted_apogee_dm;
    uint16_t target_apogee_dm;
    int16_t roll_ddeg;
    int16_t pitch_ddeg;
    int16_t yaw_ddeg;
    uint8_t deployment_percent;
    uint8_t sensor_health;
    uint16_t failed_reads;
    uint8_t message_code;
    uint8_t reserved;
} AmbarHilUsbTelemetry;

typedef struct
{
    uint16_t changed_flags;
    uint16_t current_flags;
    uint8_t previous_state;
    uint8_t current_state;
    uint8_t status_code;
    uint8_t message_code;
    uint16_t detail;
} AmbarHilUsbEvent;

typedef struct
{
    uint8_t command;
    uint8_t payload_length;
    uint8_t payload[AMBAR_HIL_USB_COMMAND_DATA_MAX];
} AmbarHilUsbCommand;

typedef struct
{
    uint16_t command_sequence;
    uint8_t command;
    uint8_t result;
    uint16_t detail;
} AmbarHilUsbAck;

/*
 * Fixed 16-byte wire payload for AMBAR_HIL_USB_PACKET_LOG_STATUS.
 *
 * This structure is encoded field-by-field by usb_comm.c, so its compiler
 * padding is never placed on the wire.
 */
typedef struct
{
    uint16_t command_sequence;
    uint8_t state;
    uint8_t error_code;
    uint32_t total_records;
    uint32_t records_sent;
    uint32_t corrupt_records;
} AmbarHilUsbLogStatus;

typedef struct
{
    uint16_t flags;
    int32_t altitude_mm;
    int32_t acceleration_mmps2;
    int32_t velocity_mmps;
    uint16_t barometer_stddev_cm;
} AmbarHilUsbSimulation;

typedef struct
{
    uint32_t actuator_inhibit_flags;
    uint32_t flight_inhibit_flags;
    int32_t target_steps;
    int32_t actual_steps;
    uint32_t driver_status;
    uint8_t machine_state;
    uint8_t flags;
    uint16_t reserved;
} AmbarHilUsbActuatorStatus;

typedef struct
{
    uint16_t simulation_sequence;
    uint16_t controller_fraction_u16;
    uint16_t actuator_target_fraction_u16;
    uint16_t xactual_fraction_u16;
    int32_t target_steps;
    int32_t actual_steps;
    uint32_t flight_inhibit_flags;
    uint32_t actuator_inhibit_flags;
    uint32_t driver_status;
    uint32_t config_crc32;
    int32_t closed_predicted_apogee_dm;
    int32_t full_predicted_apogee_dm;
    uint8_t phase;
    uint8_t machine_state;
    uint8_t state_flags;
    uint8_t feedback_source;
} AmbarHilUsbVariableHilState;

typedef struct
{
    uint8_t schema_version;
    uint8_t control_mode;
    uint8_t predictor_mode;
    uint8_t cda_point_count;
    uint32_t calibration_version;
    uint16_t target_apogee_dm;
    uint16_t mission_tolerance_dm;
    uint16_t control_deadband_cm;
    uint16_t full_deployment_error_dm;
    uint16_t minimum_deploy_altitude_dm;
    uint16_t minimum_flight_time_cs;
    uint16_t predictive_update_period_ms;
    uint16_t coast_mass_g;
    uint8_t maximum_deploy_u8;
    uint8_t hysteresis_permille;
    uint16_t deployment_cda_um2[AMBAR_HIL_USB_CDA_POINT_COUNT];
    uint16_t air_density_1e4_kgpm3;
    uint16_t density_scale_height_m;
    int16_t launch_site_elevation_dm;
    uint16_t actuator_delay_ms;
    uint16_t actuator_open_rate_milli_per_s;
    uint16_t actuator_close_rate_milli_per_s;
    uint32_t config_crc32;
} AmbarHilUsbVariableHilConfig;

typedef struct
{
    uint32_t feature_flags;
    uint16_t receive_errors;
    uint16_t transmit_drops;
} AmbarHilUsbHeartbeat;

typedef enum
{
    AMBAR_HIL_USB_RX_COMMAND,
    AMBAR_HIL_USB_RX_SIMULATION,
    AMBAR_HIL_USB_RX_VARIABLE_HIL_CONFIG_UPLOAD,
    AMBAR_HIL_USB_RX_RAW_PACKET
} AmbarHilUsbMessageKind;

typedef struct
{
    uint8_t length;
    uint8_t bytes[AMBAR_HIL_USB_MAX_PAYLOAD_SIZE];
} AmbarHilUsbRawPayload;

typedef struct
{
    AmbarHilUsbMessageKind kind;
    uint8_t packet_type;
    uint16_t sequence;
    uint32_t time_ms;
    union
    {
        AmbarHilUsbCommand command;
        AmbarHilUsbSimulation simulation;
        AmbarHilUsbVariableHilConfig variable_hil_config;
        AmbarHilUsbRawPayload raw;
    } body;
} AmbarHilUsbMessage;

typedef size_t (*AmbarHilUsbReadCallback)(void *user,
                                          uint8_t *destination,
                                          size_t capacity);
typedef size_t (*AmbarHilUsbWriteCallback)(void *user,
                                           const uint8_t *source,
                                           size_t length);
typedef uint32_t (*AmbarHilUsbTimeCallback)(void *user);
typedef bool (*AmbarHilUsbConnectedCallback)(void *user);

typedef struct
{
    AmbarHilUsbReadCallback read;
    AmbarHilUsbWriteCallback write;
    AmbarHilUsbTimeCallback now_ms;
    AmbarHilUsbConnectedCallback is_connected;
    void *user;
} AmbarHilUsbIo;

typedef struct
{
    uint32_t received_frames;
    uint32_t receive_errors;
    uint32_t receive_queue_drops;
    uint32_t transmitted_frames;
    uint32_t transmit_drops;
    uint32_t io_errors;
    uint32_t connection_count;
    uint32_t disconnection_count;
    uint8_t receive_queue_count;
    uint8_t transmit_queue_count;
    bool connected;
} AmbarHilUsbStats;

/* Public only so callers can allocate the context statically; treat as opaque. */
typedef struct
{
    uint8_t bytes[AMBAR_HIL_USB_MAX_FRAME_SIZE];
    uint8_t length;
    uint8_t offset;
} AmbarHilUsbTxFrame;

typedef struct
{
    AmbarHilUsbIo io;
    AmbarHilUsbStats stats;
    uint16_t transmit_sequence;
    uint8_t encoded_receive[AMBAR_HIL_USB_MAX_FRAME_SIZE - 1u];
    uint8_t encoded_receive_length;
    bool drop_until_delimiter;
    AmbarHilUsbMessage receive_queue[AMBAR_HIL_USB_RX_QUEUE_DEPTH];
    uint8_t receive_head;
    uint8_t receive_tail;
    uint8_t receive_count;
    AmbarHilUsbMessage priority_estop;
    bool priority_estop_pending;
    AmbarHilUsbTxFrame transmit_queue[AMBAR_HIL_USB_TX_QUEUE_DEPTH];
    uint8_t transmit_head;
    uint8_t transmit_tail;
    uint8_t transmit_count;
    AmbarHilUsbTxFrame priority_transmit;
    bool priority_transmit_pending;
    bool initialized;
    bool connection_observed;
    bool connected;
} AmbarHilUsb;

/* Validate callbacks and initialize a caller-owned, fixed-storage context. */
bool AmbarHilUsb_Init(AmbarHilUsb *context, const AmbarHilUsbIo *io);

/* Discard partial/queued traffic and restart the transmit sequence at zero. */
void AmbarHilUsb_Reset(AmbarHilUsb *context);

/* Service connection state, bounded RX work, and one nonblocking TX write. */
void AmbarHilUsb_Poll(AmbarHilUsb *context);

/* ESTOP is returned before the normal FIFO; this function never executes it. */
bool AmbarHilUsb_TakeMessage(AmbarHilUsb *context,
                             AmbarHilUsbMessage *message);

bool AmbarHilUsb_IsConnected(const AmbarHilUsb *context);
AmbarHilUsbStats AmbarHilUsb_GetStats(const AmbarHilUsb *context);

/* Generic packet senders copy and frame payload bytes immediately. */
bool AmbarHilUsb_SendPacket(AmbarHilUsb *context,
                            uint8_t packet_type,
                            const uint8_t *payload,
                            size_t payload_length);
bool AmbarHilUsb_SendCorrelatedPacket(AmbarHilUsb *context,
                                      uint8_t packet_type,
                                      uint16_t sequence,
                                      const uint8_t *payload,
                                      size_t payload_length);

bool AmbarHilUsb_SendAck(AmbarHilUsb *context, const AmbarHilUsbAck *payload);
bool AmbarHilUsb_SendTelemetry(AmbarHilUsb *context,
                               const AmbarHilUsbTelemetry *payload);
bool AmbarHilUsb_SendEvent(AmbarHilUsb *context,
                           const AmbarHilUsbEvent *payload);
bool AmbarHilUsb_SendActuatorStatus(AmbarHilUsb *context,
                                    const AmbarHilUsbActuatorStatus *payload);
bool AmbarHilUsb_SendHeartbeat(AmbarHilUsb *context,
                               const AmbarHilUsbHeartbeat *payload);
bool AmbarHilUsb_SendVariableHilState(
    AmbarHilUsb *context,
    uint16_t correlated_sequence,
    const AmbarHilUsbVariableHilState *payload);
bool AmbarHilUsb_SendVariableHilConfig(
    AmbarHilUsb *context,
    uint16_t correlated_sequence,
    const AmbarHilUsbVariableHilConfig *payload);

/* Four two-bit health values packed in IMU/barometer/magnetometer/actuator order. */
uint8_t AmbarHilUsb_PackSensorHealth(uint8_t imu,
                                     uint8_t barometer,
                                     uint8_t magnetometer,
                                     uint8_t actuator);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_HIL_USB_H */
