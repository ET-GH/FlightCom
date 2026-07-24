/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

	/*
	 * Main is the highest level of abstraction. Handling configuration and subsystem connection.
	 * the abstraction hierarchy goes like this:
	 * Main --> Behavior --> rocket_sensors --> sensor drivers -> communication protocol
	 * 		|			 |--> Fusion
	 * 		|			 |--> ekf
	 * 		|			 |--> controller
	 * 		|			 |--> airbrake --> motor driver --> communication protocol
	 * 		|--> Radio_bridge --> radio module --> communication protocol
	 * 		| <Not implemented below>
	 * 		|--> Memory organization --> communication protocol
	 * 		|--> usb communication
	 *
	 * AI generated most of the comments and documentation.
	 * I did the verification though, it should be all clear and easy to understand.
	 * behavior.c was reformatted greatly though, it was not well organized.
	 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_usbx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "radio_bridge.h"
#include "rocket_protocol.h"
//#include "rocket_sensors.h"
//#include "lsm6dsv32x.h"
//#include "lis2mdl.h"
//#include "bmp388.h"
//#include "tmc5240.h"
#include "airbrake.h"
#include "w25q64.h"
//#include "controller.h"
#include "usb_comm.h"
//#include "altitude_ekf.h"
//#include "Fusion.h"
#include "behavior.h"
// only for one test
//#include "openrocket_run_data.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * Command-driven USB archive export state.
 *
 * USB enumeration and PING remain available in every state. Flash records are
 * transmitted only after the host sends AMBAR_HIL_USB_COMMAND_EXPORT_LOG.
 */
typedef enum
{
    MEMORY_EXPORT_IDLE = 0,
    MEMORY_EXPORT_BEGIN_PENDING,
    MEMORY_EXPORT_SENDING,
    MEMORY_EXPORT_COMPLETE_PENDING,
    MEMORY_EXPORT_CANCEL_PENDING,
    MEMORY_EXPORT_ERROR_PENDING
} MemoryExportState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ENABLE_AIRBRAKE_BENCH_OVERRIDE  0U

/*
 * Store one telemetry sample every 250 ms. Event records are inserted
 * immediately when flags, state, status, or deployment state changes.
 * With no events, this provides about 12 hours of telemetry capacity.
 */
#define MEMORY_LOG_PERIOD_MS                 250U

/*
 * Archived USB telemetry uses the protocol payload's reserved byte:
 *   bit 7    = this sample came from flash rather than live telemetry
 *   bits 0-6 = elapsed time since the previous archived sample, in 10 ms units
 *
 * The first archived sample carries a zero delta. The ground station can
 * reconstruct a flight-relative CSV time column without changing the AMBAR
 * framing layer or stealing any telemetry measurement field.
 */
#define MEMORY_ARCHIVE_FLAG                  0x80U
#define MEMORY_ARCHIVE_DELTA_MASK            0x7FU
#define MEMORY_ARCHIVE_DELTA_QUANTUM_MS      10U
#define MEMORY_ARCHIVE_EVENT_FLAG            0x8000U
#define MEMORY_ARCHIVE_EVENT_DELTA_MASK      0x7FFFU


/*
 * Archive-transfer control events.
 *
 * These are ordinary (non-archived) EVENT packets, so the GroundStation can
 * use them to determine the exact record count and transfer completion without
 * writing protocol-control rows into the exported flight CSV.
 *
 * EVENT layout for these two message codes:
 *   changed_flags  = record count bits 0..15
 *   current_flags  = record count bits 16..31
 *   previous_state = control-envelope version
 *   current_state  = transfer result
 */
#define MEMORY_ARCHIVE_CONTROL_BEGIN         0xF0U
#define MEMORY_ARCHIVE_CONTROL_END           0xF1U
#define MEMORY_ARCHIVE_CONTROL_VERSION       1U
#define MEMORY_ARCHIVE_RESULT_OK             0U
#define MEMORY_ARCHIVE_RESULT_FLASH_ERROR    1U
#define MEMORY_ARCHIVE_RESULT_READ_ERROR     2U
#define MEMORY_EXPORT_PHASE_IDLE             0U
#define MEMORY_EXPORT_PHASE_BEGIN            1U
#define MEMORY_EXPORT_PHASE_RECORDS          2U
#define MEMORY_EXPORT_PHASE_END              3U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
BehaviorConfig_t g_behavior_config;
volatile const BehaviorTelemetry_t *g_behavior_telemetry = NULL;

/* Debugger-controlled one-shot commands. */
volatile bool g_request_comprehensive_test = false;
volatile bool g_request_standard_mode = false;
volatile bool g_apply_behavior_config = false;

/*
 * Command-controlled application gates.
 *
 * The radio bridge always keeps command interpretation alive. Main uses these
 * gates to implement the requested standby and flight-computer semantics
 * without moving application policy into the packet parser.
 */
volatile bool g_flight_computer_enabled = true;
volatile bool g_measurement_updates_enabled = true;
/*
 * Protocol-level airbrake enable state.
 *
 * true:
 *   Automatic or manually requested airbrake operation is available.
 *
 * false:
 *   Automatic control is disabled, new relative movements are blocked, and
 *   behavior.c has commanded the safe fully retracted 0% position.
 */
volatile bool g_airbrakes_enabled = true;


/* ------------------------------------------------------------------------- */
/* External W25Q64JV flash and Rocket Protocol archive diagnostics            */
/* ------------------------------------------------------------------------- */

static W25Q64_HandleTypeDef g_memory_flash;

/* These variables are intentionally visible to Live Expressions/debugger. */
volatile W25Q64_Result_t g_memory_flash_init_result = W25Q64_ERROR;
volatile W25Q64_Result_t g_memory_log_result = W25Q64_ERROR;
volatile uint32_t g_memory_log_records = 0U;
volatile uint32_t g_memory_log_capacity = 0U;
volatile uint32_t g_memory_log_corrupt_records = 0U;
volatile uint32_t g_memory_log_write_failures = 0U;
volatile uint32_t g_memory_exported_records = 0U;
volatile uint32_t g_memory_export_read_failures = 0U;
volatile bool g_memory_export_active = false;

/*
 * Set true in the debugger to erase the archive. The request is accepted only
 * while measurement updates are suspended and no USB export is active because
 * erasing all 2047 log sectors is intentionally blocking and can take a long
 * time. The flag stays true until those safety conditions are met.
 */
volatile bool g_request_memory_log_erase = false;

static uint32_t g_memory_last_log_ms = 0U;
static MemoryExportState_t g_memory_export_state = MEMORY_EXPORT_IDLE;
static uint16_t g_memory_export_command_sequence = 0U;
static uint8_t g_memory_export_error_code = 0U;
static bool g_memory_usb_was_connected = false;
static bool g_memory_export_record_loaded = false;
static uint32_t g_memory_export_index = 0U;
static uint32_t g_memory_export_total = 0U;
static uint32_t g_memory_export_record_time_ms = 0U;
static uint32_t g_memory_export_previous_time_ms = 0U;
static uint8_t g_memory_export_packet_type = ROCKET_PKT_TELEMETRY;
static RocketTelemetryPayload g_memory_export_payload;
static RocketEventPayload g_memory_export_event;

static uint8_t g_memory_export_phase = MEMORY_EXPORT_PHASE_IDLE;
static uint8_t g_memory_export_result = MEMORY_ARCHIVE_RESULT_OK;

/* Previous archived state used to create event records at their exact time. */
static bool g_memory_event_state_valid = false;
static uint16_t g_memory_previous_flags = 0U;
static uint8_t g_memory_previous_state = 0U;
static uint8_t g_memory_previous_status = ROCKET_STATUS_UNKNOWN;
static uint8_t g_memory_previous_deployment_percent = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Strong implementation of the weak command callback in radio_bridge.c.
 * The bridge has already validated command length and value ranges before this
 * function is called.
 */
uint8_t RadioBridge_ExecuteCommand(uint8_t command,
                                   const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint32_t now_ms,
                                   uint32_t *detail);

void MX_USB_PCD_Init(void);

/*
 * High-level log API implemented at the end of w25q64.c. Keeping these
 * declarations local lets this integration replace only main.c and w25q64.c.
 * They can be moved into w25q64.h later without changing the implementation.
 */
W25Q64_Result_t W25Q64_LogOpen(W25Q64_HandleTypeDef *dev);
W25Q64_Result_t W25Q64_LogAppend(
    W25Q64_HandleTypeDef *dev,
    uint32_t time_ms,
    const RocketTelemetryPayload *payload);
W25Q64_Result_t W25Q64_LogAppendEvent(
    W25Q64_HandleTypeDef *dev,
    uint32_t time_ms,
    const RocketEventPayload *payload);
W25Q64_Result_t W25Q64_LogReadRecord(
    W25Q64_HandleTypeDef *dev,
    uint32_t record_index,
    uint32_t *time_ms,
    uint8_t *packet_type,
    RocketTelemetryPayload *telemetry,
    RocketEventPayload *event);
W25Q64_Result_t W25Q64_LogErase(W25Q64_HandleTypeDef *dev);
uint32_t W25Q64_LogGetCount(void);
uint32_t W25Q64_LogGetCapacity(void);
uint32_t W25Q64_LogGetCorruptCount(void);
bool W25Q64_LogCanAppend(void);

static void Memory_LogTask(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    bool behavior_updated);
static void Memory_ExportTask(void);
static void Memory_ServiceEraseRequest(void);
static bool Memory_ExportAllowed(void);
static bool Memory_SendExportStatus(uint8_t state, uint8_t error_code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================================================================== */
/* External flash logging and USB replay                                      */
/* ========================================================================== */

static int16_t Memory_ClampI16(float value)
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

static uint16_t Memory_ClampU16(float value)
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

static bool Memory_SensorStatusIsFault(HAL_StatusTypeDef status)
{
    return (status != HAL_OK) && (status != HAL_BUSY);
}

static uint16_t Memory_MakeFlags(
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

    if (Memory_SensorStatusIsFault(telemetry->imu_status) ||
        Memory_SensorStatusIsFault(telemetry->mag_status) ||
        Memory_SensorStatusIsFault(telemetry->baro_status))
    {
        flags |= ROCKET_FLAG_SENSOR_FAULT;
    }

    return flags;
}

static uint8_t Memory_MakeState(
    const volatile BehaviorTelemetry_t *telemetry)
{
    return (uint8_t)((((uint8_t)telemetry->mode & 0x0FU) << 4) |
                     ((uint8_t)telemetry->flight_phase & 0x0FU));
}

static uint8_t Memory_MakeSensorHealth(
    const volatile BehaviorTelemetry_t *telemetry)
{
    const uint8_t imu_health =
        Memory_SensorStatusIsFault(telemetry->imu_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;
    const uint8_t mag_health =
        Memory_SensorStatusIsFault(telemetry->mag_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;
    const uint8_t baro_health =
        Memory_SensorStatusIsFault(telemetry->baro_status)
            ? ROCKET_SENSOR_FAULT
            : ROCKET_SENSOR_OK;

    return (uint8_t)(imu_health |
                     (uint8_t)(mag_health << 2) |
                     (uint8_t)(baro_health << 4));
}

static uint8_t Memory_MapStatus(uint32_t status)
{
    switch (status)
    {
        case BEHAVIOR_STATUS_OK:
            return ROCKET_STATUS_OK;
        case BEHAVIOR_STATUS_SENSOR_INIT_FAILED:
            return ROCKET_STATUS_SENSOR_INIT_FAILED;
        case BEHAVIOR_STATUS_SENSOR_READ_FAILED:
            return ROCKET_STATUS_SENSOR_READ_FAILED;
        case BEHAVIOR_STATUS_UNSUPPORTED_MODE:
            return ROCKET_STATUS_UNSUPPORTED_MODE;
        case BEHAVIOR_STATUS_BAD_ARGUMENT:
            return ROCKET_STATUS_BAD_ARGUMENT;
        case BEHAVIOR_STATUS_INVALID_CONFIG:
            return ROCKET_STATUS_INVALID_CONFIG;
        default:
            return ROCKET_STATUS_UNKNOWN;
    }
}

static bool Memory_BuildRocketTelemetry(
    const volatile BehaviorTelemetry_t *telemetry,
    RocketTelemetryPayload *payload)
{
    uint32_t failed_reads;

    if ((telemetry == NULL) || (payload == NULL))
    {
        return false;
    }

    failed_reads = telemetry->failed_reads;
    if (failed_reads > 65535U)
    {
        failed_reads = 65535U;
    }

    memset(payload, 0, sizeof(*payload));
    payload->flags = Memory_MakeFlags(telemetry);
    payload->state = Memory_MakeState(telemetry);
    payload->status_code = Memory_MapStatus((uint32_t)telemetry->status);
    payload->altitude_dm =
        Memory_ClampI16(telemetry->ekf_altitude_m * 10.0f);
    payload->velocity_cms =
        Memory_ClampI16(telemetry->ekf_velocity_m_s * 100.0f);
    payload->acceleration_cms2 =
        Memory_ClampI16(telemetry->ekf_acceleration_m_s2 * 100.0f);
    payload->predicted_apogee_dm =
        Memory_ClampU16(telemetry->predicted_apogee_m * 10.0f);
    payload->target_apogee_dm =
        Memory_ClampU16(telemetry->target_apogee_m * 10.0f);
    payload->roll_ddeg =
        Memory_ClampI16(telemetry->fusion_roll_deg * 10.0f);
    payload->pitch_ddeg =
        Memory_ClampI16(telemetry->fusion_pitch_deg * 10.0f);
    payload->yaw_ddeg =
        Memory_ClampI16(telemetry->fusion_yaw_deg * 10.0f);
    payload->deployment_percent =
        telemetry->controller_requested_percent;
    payload->sensor_health = Memory_MakeSensorHealth(telemetry);
    payload->failed_reads = (uint16_t)failed_reads;
    payload->message_code = ROCKET_MSG_NONE;
    payload->reserved = 0U;

    return true;
}

static uint8_t Memory_SelectEventMessage(
    bool previous_valid,
    uint16_t previous_flags,
    uint16_t current_flags,
    uint8_t previous_state,
    uint8_t current_state,
    uint8_t previous_deployment,
    uint8_t current_deployment)
{
    const uint8_t previous_phase = previous_state & 0x0FU;
    const uint8_t current_phase = current_state & 0x0FU;

    if (!previous_valid)
    {
        return ROCKET_MSG_BOOT;
    }

    if (previous_phase != current_phase)
    {
        switch (current_phase)
        {
            case BEHAVIOR_FLIGHT_PHASE_POWERED_ASCENT:
                return ROCKET_MSG_LAUNCH_DETECTED;
            case BEHAVIOR_FLIGHT_PHASE_COAST:
                return ROCKET_MSG_BURNOUT_DETECTED;
            case BEHAVIOR_FLIGHT_PHASE_DESCENT:
                return ROCKET_MSG_APOGEE_REACHED;
            case BEHAVIOR_FLIGHT_PHASE_LANDED:
                return ROCKET_MSG_LANDING_DETECTED;
            default:
                break;
        }
    }

    if (((previous_flags ^ current_flags) &
         ROCKET_FLAG_CONTROLLER_ENABLED) != 0U)
    {
        return ((current_flags & ROCKET_FLAG_CONTROLLER_ENABLED) != 0U)
            ? ROCKET_MSG_CONTROLLER_ENABLED
            : ROCKET_MSG_CONTROLLER_DISABLED;
    }

    if ((previous_deployment == 0U) && (current_deployment != 0U))
    {
        return ROCKET_MSG_AIRBRAKE_DEPLOYED;
    }

    if ((previous_deployment != 0U) && (current_deployment == 0U))
    {
        return ROCKET_MSG_AIRBRAKE_RETRACTED;
    }

    if ((previous_state & 0xF0U) != (current_state & 0xF0U))
    {
        return ROCKET_MSG_MODE_CHANGED;
    }

    return ROCKET_MSG_NONE;
}

static void Memory_RocketToUsbTelemetry(
    const RocketTelemetryPayload *rocket,
    AmbarHilUsbTelemetry *usb)
{
    memset(usb, 0, sizeof(*usb));
    usb->flags = rocket->flags;
    usb->state = rocket->state;
    usb->status_code = rocket->status_code;
    usb->altitude_dm = rocket->altitude_dm;
    usb->velocity_cms = rocket->velocity_cms;
    usb->acceleration_cms2 = rocket->acceleration_cms2;
    usb->predicted_apogee_dm = rocket->predicted_apogee_dm;
    usb->target_apogee_dm = rocket->target_apogee_dm;
    usb->roll_ddeg = rocket->roll_ddeg;
    usb->pitch_ddeg = rocket->pitch_ddeg;
    usb->yaw_ddeg = rocket->yaw_ddeg;
    usb->deployment_percent = rocket->deployment_percent;
    usb->sensor_health = rocket->sensor_health;
    usb->failed_reads = rocket->failed_reads;
    usb->message_code = rocket->message_code;
    usb->reserved = rocket->reserved;
}

static void Memory_RocketToUsbEvent(
    const RocketEventPayload *rocket,
    AmbarHilUsbEvent *usb)
{
    memset(usb, 0, sizeof(*usb));
    usb->changed_flags = rocket->changed_flags;
    usb->current_flags = rocket->current_flags;
    usb->previous_state = rocket->previous_state;
    usb->current_state = rocket->current_state;
    usb->status_code = rocket->status_code;
    usb->message_code = rocket->message_code;
    usb->detail = rocket->detail;
}


/*
 * Flash export is blocked only during active ascent/coast/descent.
 *
 * This still permits retrieval after a reboot, during preflight bench work,
 * after landing, while in standby, or while flight-computer updates are
 * disabled. USB PING remains available regardless of this result.
 */
static bool Memory_ExportAllowed(void)
{
    uint8_t phase;

    if (!g_flight_computer_enabled || !g_measurement_updates_enabled)
    {
        return true;
    }

    if (g_behavior_telemetry == NULL)
    {
        return false;
    }

    phase = (uint8_t)g_behavior_telemetry->flight_phase;

    return (phase != BEHAVIOR_FLIGHT_PHASE_POWERED_ASCENT) &&
           (phase != BEHAVIOR_FLIGHT_PHASE_COAST) &&
           (phase != BEHAVIOR_FLIGHT_PHASE_DESCENT);
}

/*
 * Queue one explicit archive-transfer status packet.
 *
 * The host uses STARTED.total_records for the progress-bar maximum and uses
 * COMPLETE/CANCELLED/ERROR instead of guessing completion from a timeout.
 */
static bool Memory_SendExportStatus(uint8_t state, uint8_t error_code)
{
    AmbarHilUsbLogStatus status;

    memset(&status, 0, sizeof(status));
    status.command_sequence = g_memory_export_command_sequence;
    status.state = state;
    status.error_code = error_code;
    status.total_records = g_memory_export_total;
    status.records_sent = g_memory_exported_records;
    status.corrupt_records = W25Q64_LogGetCorruptCount();

    return USBComm_SendLogStatus(&status);
}

/*
 * Strong implementation of the weak application callback in usb_comm.c.
 *
 * The USB transport validates framing and payload extraction. This callback
 * owns only the application policy and state transition.
 */
uint8_t USBComm_ExecuteCommand(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length,
    uint16_t command_sequence,
    uint16_t *detail)
{
    (void)payload;

    if (detail == NULL)
    {
        return AMBAR_HIL_USB_ACK_EXECUTION_ERROR;
    }

    *detail = 0U;

    switch (command)
    {
        case AMBAR_HIL_USB_COMMAND_EXPORT_LOG:
        {
            if (payload_length != 0U)
            {
                return AMBAR_HIL_USB_ACK_BAD_LENGTH;
            }

            if (g_memory_flash_init_result != W25Q64_OK)
            {
                return AMBAR_HIL_USB_ACK_EXECUTION_ERROR;
            }

            if (g_memory_export_state != MEMORY_EXPORT_IDLE)
            {
                return AMBAR_HIL_USB_ACK_BUSY;
            }

            if (!Memory_ExportAllowed())
            {
                return AMBAR_HIL_USB_ACK_BUSY;
            }

            /*
             * Freeze the record total at command acceptance. Memory_LogTask()
             * sees the non-idle state before the next append opportunity.
             */
            g_memory_export_command_sequence = command_sequence;
            g_memory_export_index = 0U;
            g_memory_export_total = W25Q64_LogGetCount();
            g_memory_export_previous_time_ms = 0U;
            g_memory_exported_records = 0U;
            g_memory_export_record_loaded = false;
            g_memory_export_error_code = 0U;
            g_memory_export_active = true;
            g_memory_export_state = MEMORY_EXPORT_BEGIN_PENDING;

            *detail = (g_memory_export_total > 65535U)
                ? 65535U
                : (uint16_t)g_memory_export_total;

            return AMBAR_HIL_USB_ACK_OK;
        }

        case AMBAR_HIL_USB_COMMAND_CANCEL_EXPORT:
        {
            if (payload_length != 0U)
            {
                return AMBAR_HIL_USB_ACK_BAD_LENGTH;
            }

            if (g_memory_export_state == MEMORY_EXPORT_IDLE)
            {
                return AMBAR_HIL_USB_ACK_BAD_VALUE;
            }

            g_memory_export_state = MEMORY_EXPORT_CANCEL_PENDING;

            *detail = (g_memory_exported_records > 65535U)
                ? 65535U
                : (uint16_t)g_memory_exported_records;

            return AMBAR_HIL_USB_ACK_OK;
        }

        default:
            return AMBAR_HIL_USB_ACK_UNSUPPORTED;
    }
}

/**
 * Detect events every behavior iteration and store telemetry every 250 ms.
 * Event timestamps therefore are not rounded to the telemetry interval.
 */
static void Memory_LogTask(
    const volatile BehaviorTelemetry_t *telemetry,
    uint32_t now_ms,
    bool behavior_updated)
{
    RocketTelemetryPayload sample;
    const bool ready =
        behavior_updated &&
        (telemetry != NULL) &&
        (g_memory_flash_init_result == W25Q64_OK) &&
        W25Q64_LogCanAppend() &&
        (g_memory_export_state == MEMORY_EXPORT_IDLE);

    if (!ready || !Memory_BuildRocketTelemetry(telemetry, &sample))
    {
        return;
    }

    {
        const bool deployment_boundary_changed =
            g_memory_event_state_valid &&
            ((g_memory_previous_deployment_percent == 0U) !=
             (sample.deployment_percent == 0U));
        const bool event_needed =
            !g_memory_event_state_valid ||
            (sample.flags != g_memory_previous_flags) ||
            (sample.state != g_memory_previous_state) ||
            (sample.status_code != g_memory_previous_status) ||
            deployment_boundary_changed;

        if (event_needed)
        {
            RocketEventPayload event;

            memset(&event, 0, sizeof(event));
            event.changed_flags = g_memory_event_state_valid
                ? (uint16_t)(sample.flags ^ g_memory_previous_flags)
                : sample.flags;
            event.current_flags = sample.flags;
            event.previous_state = g_memory_event_state_valid
                ? g_memory_previous_state
                : sample.state;
            event.current_state = sample.state;
            event.status_code = sample.status_code;
            event.message_code = Memory_SelectEventMessage(
                g_memory_event_state_valid,
                g_memory_previous_flags,
                sample.flags,
                event.previous_state,
                event.current_state,
                g_memory_previous_deployment_percent,
                sample.deployment_percent);
            event.detail = 0U;

            g_memory_log_result =
                W25Q64_LogAppendEvent(&g_memory_flash,
                                      now_ms,
                                      &event);

            if (g_memory_log_result != W25Q64_OK)
            {
                ++g_memory_log_write_failures;
                return;
            }

            g_memory_previous_flags = sample.flags;
            g_memory_previous_state = sample.state;
            g_memory_previous_status = sample.status_code;
            g_memory_previous_deployment_percent =
                sample.deployment_percent;
            g_memory_event_state_valid = true;
        }
    }

    if ((uint32_t)(now_ms - g_memory_last_log_ms) < MEMORY_LOG_PERIOD_MS)
    {
        g_memory_log_records = W25Q64_LogGetCount();
        return;
    }

    g_memory_last_log_ms = now_ms;
    g_memory_log_result =
        W25Q64_LogAppend(&g_memory_flash, now_ms, &sample);

    if (g_memory_log_result != W25Q64_OK)
    {
        ++g_memory_log_write_failures;
    }

    g_memory_log_records = W25Q64_LogGetCount();
    g_memory_log_corrupt_records = W25Q64_LogGetCorruptCount();
}


/**
 * Queue a transfer-control EVENT without the archive marker.
 *
 * Because detail bit 15 is clear, AmbarUsbClient::startArchiveCsv() filters this
 * packet out of the CSV while RawImportController can still inspect it through
 * packetReceived(). This keeps the CSV limited to actual flash records.
 */
static bool Memory_SendArchiveControl(uint8_t message_code,
                                      uint32_t record_count,
                                      uint8_t result_code)
{
    AmbarHilUsbEvent control;

    memset(&control, 0, sizeof(control));
    control.changed_flags =
        (uint16_t)(record_count & 0xFFFFUL);
    control.current_flags =
        (uint16_t)((record_count >> 16) & 0xFFFFUL);
    control.previous_state = MEMORY_ARCHIVE_CONTROL_VERSION;
    control.current_state = result_code;
    control.status_code =
        (result_code == MEMORY_ARCHIVE_RESULT_OK)
            ? ROCKET_STATUS_OK
            : ROCKET_STATUS_UNKNOWN;
    control.message_code = message_code;

    /*
     * Keep the archive marker clear. The packet is transfer metadata, not a
     * stored flight event, and therefore must not become a CSV row.
     */
    control.detail = 0U;

    return USBComm_SendEvent(&control);
}

/**
 * Replay telemetry and event records only after an EXPORT_LOG USB command.
 *
 * Telemetry uses reserved bit 7 plus a 10 ms delta, as before. Archived events
 * use detail bit 15 plus a 10 ms delta. The generated flash events currently
 * store detail=0, so no event information is lost by this USB convention.
 */

static void Memory_ExportTask(void)
{
    const bool connected = USBComm_IsConnected();

    if (!connected)
    {
        g_memory_usb_was_connected = false;
        g_memory_export_active = false;
        g_memory_export_record_loaded = false;
        g_memory_export_phase = MEMORY_EXPORT_PHASE_IDLE;
        g_memory_export_result = MEMORY_ARCHIVE_RESULT_OK;
        return;
    }

    if (!g_memory_usb_was_connected)
    {
        g_memory_usb_was_connected = true;
        g_memory_export_index = 0U;
        g_memory_export_total =
            (g_memory_flash_init_result == W25Q64_OK)
                ? W25Q64_LogGetCount()
                : 0U;
        g_memory_export_previous_time_ms = 0U;
        g_memory_exported_records = 0U;
        g_memory_export_record_loaded = false;
        g_memory_export_result =
            (g_memory_flash_init_result == W25Q64_OK)
                ? MEMORY_ARCHIVE_RESULT_OK
                : MEMORY_ARCHIVE_RESULT_FLASH_ERROR;
        g_memory_export_phase = MEMORY_EXPORT_PHASE_BEGIN;
        g_memory_export_active = true;
    }

    if (!g_memory_export_active)
    {
        return;
    }

    /*
     * Send the total record count before the first archived row. Queue pressure
     * is handled by retrying this phase on the next cooperative-loop iteration.
     */
    if (g_memory_export_phase == MEMORY_EXPORT_PHASE_BEGIN)
    {
        if (Memory_SendArchiveControl(MEMORY_ARCHIVE_CONTROL_BEGIN,
                                      g_memory_export_total,
                                      g_memory_export_result))
        {
            g_memory_export_phase =
                ((g_memory_export_result == MEMORY_ARCHIVE_RESULT_OK) &&
                 (g_memory_export_total != 0U))
                    ? MEMORY_EXPORT_PHASE_RECORDS
                    : MEMORY_EXPORT_PHASE_END;
        }

        return;
    }

    if (g_memory_export_phase == MEMORY_EXPORT_PHASE_RECORDS)
    {
        if (!g_memory_export_record_loaded)
        {
            g_memory_log_result =
                W25Q64_LogReadRecord(&g_memory_flash,
                                     g_memory_export_index,
                                     &g_memory_export_record_time_ms,
                                     &g_memory_export_packet_type,
                                     &g_memory_export_payload,
                                     &g_memory_export_event);

            if (g_memory_log_result != W25Q64_OK)
            {
                ++g_memory_export_read_failures;
                g_memory_log_corrupt_records =
                    W25Q64_LogGetCorruptCount();
                g_memory_export_result =
                    MEMORY_ARCHIVE_RESULT_READ_ERROR;
                g_memory_export_record_loaded = false;
                g_memory_export_phase = MEMORY_EXPORT_PHASE_END;
                return;
            }

            g_memory_export_record_loaded = true;
        }

        {
            uint32_t delta_ms = 0U;
            uint32_t delta_units = 0U;
            bool queued = false;

            if (g_memory_export_index != 0U)
            {
                delta_ms =
                    g_memory_export_record_time_ms -
                    g_memory_export_previous_time_ms;
                delta_units =
                    (delta_ms +
                     (MEMORY_ARCHIVE_DELTA_QUANTUM_MS / 2U)) /
                    MEMORY_ARCHIVE_DELTA_QUANTUM_MS;
            }

            if (g_memory_export_packet_type == ROCKET_PKT_TELEMETRY)
            {
                AmbarHilUsbTelemetry usb_payload;

                Memory_RocketToUsbTelemetry(
                    &g_memory_export_payload,
                    &usb_payload);

                if (delta_units > MEMORY_ARCHIVE_DELTA_MASK)
                {
                    delta_units = MEMORY_ARCHIVE_DELTA_MASK;
                }

                usb_payload.reserved =
                    (uint8_t)(MEMORY_ARCHIVE_FLAG |
                              (uint8_t)delta_units);
                queued = USBComm_SendTelemetry(&usb_payload);
            }
            else if (g_memory_export_packet_type == ROCKET_PKT_EVENT)
            {
                AmbarHilUsbEvent usb_event;

                Memory_RocketToUsbEvent(
                    &g_memory_export_event,
                    &usb_event);

                if (delta_units > MEMORY_ARCHIVE_EVENT_DELTA_MASK)
                {
                    delta_units =
                        MEMORY_ARCHIVE_EVENT_DELTA_MASK;
                }

                usb_event.detail =
                    (uint16_t)(MEMORY_ARCHIVE_EVENT_FLAG |
                               (uint16_t)delta_units);
                queued = USBComm_SendEvent(&usb_event);
            }
            else
            {
                ++g_memory_export_read_failures;
                g_memory_export_result =
                    MEMORY_ARCHIVE_RESULT_READ_ERROR;
                g_memory_export_record_loaded = false;
                g_memory_export_phase = MEMORY_EXPORT_PHASE_END;
                return;
            }

            if (queued)
            {
                g_memory_export_previous_time_ms =
                    g_memory_export_record_time_ms;
                ++g_memory_export_index;
                ++g_memory_exported_records;
                g_memory_export_record_loaded = false;

                if (g_memory_export_index >=
                    g_memory_export_total)
                {
                    g_memory_export_phase =
                        MEMORY_EXPORT_PHASE_END;
                }
            }
        }

        return;
    }

    /*
     * Send an explicit terminal count/result. The GroundStation closes the CSV
     * and loads the analysis graphs only after receiving this packet.
     */
    if (g_memory_export_phase == MEMORY_EXPORT_PHASE_END)
    {
        if (Memory_SendArchiveControl(MEMORY_ARCHIVE_CONTROL_END,
                                      g_memory_exported_records,
                                      g_memory_export_result))
        {
            g_memory_export_phase = MEMORY_EXPORT_PHASE_IDLE;
            g_memory_export_active = false;
        }
    }
}


/** Execute the intentionally blocking erase only in a safe, suspended state. */
static void Memory_ServiceEraseRequest(void)
{
    if (!g_request_memory_log_erase ||
        g_measurement_updates_enabled ||
        g_memory_export_active ||
        (g_memory_flash_init_result != W25Q64_OK))
    {
        return;
    }

    g_request_memory_log_erase = false;
    g_memory_log_result = W25Q64_LogErase(&g_memory_flash);
    g_memory_log_records = W25Q64_LogGetCount();
    g_memory_log_capacity = W25Q64_LogGetCapacity();
    g_memory_log_corrupt_records = W25Q64_LogGetCorruptCount();
    g_memory_event_state_valid = false;
    g_memory_last_log_ms = HAL_GetTick();
}

/**
 * @brief Execute one validated command extracted by radio_bridge.c.
 *
 * Packet parsing remains inside the radio bridge. This callback contains only
 * application policy and calls existing behavior/airbrake APIs, which keeps the
 * change localized and avoids teaching main.c the protocol wire format.
 *
 * @return One RocketAckCode value for the command acknowledgement.
 */
uint8_t RadioBridge_ExecuteCommand(uint8_t command,
                                   const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint32_t now_ms,
                                   uint32_t *detail)
{
    if ((payload == NULL) || (detail == NULL))
    {
        return ROCKET_ACK_EXECUTION_ERROR;
    }

    *detail = 0U;

    switch (command)
    {
        case ROCKET_CMD_SET_TARGET_APOGEE:
        {
            const uint16_t target_decimeters =
                RocketProtocol_ReadU16(payload);

            if (Behavior_SetTargetApogee(
                    (float)target_decimeters / 10.0f,
                    now_ms) != BEHAVIOR_STATUS_OK)
            {
                return ROCKET_ACK_BAD_VALUE;
            }

            *detail = target_decimeters;
            return ROCKET_ACK_OK;
        }

        case ROCKET_CMD_SET_CONTROLLER:
        {
            const bool enabled = (payload[0] != 0U);

            /*
             * Do not change controller ownership while a relative motor-step
             * movement is active. Percentage and automatic commands must wait
             * until the deferred motor-step acknowledgement is returned.
             */
            if (Airbrake_IsManualStepActive())
            {
                return ROCKET_ACK_BUSY;
            }

            /*
             * SET_CONTROLLER,1 enables automatic airbrake control.
             *
             * SET_CONTROLLER,0 disables automatic control and commands the
             * safe fully retracted 0% position through behavior.c.
             */
            if (Behavior_SetControllerEnabled(enabled) !=
                BEHAVIOR_STATUS_OK)
            {
                return ROCKET_ACK_EXECUTION_ERROR;
            }

            g_airbrakes_enabled = enabled;

            /*
             * Automatic and disabled modes do not use manual override.
             * A later manual percentage command explicitly enables it again.
             */
            Airbrake_SetManualOverride(false);

            *detail = payload[0];
            return ROCKET_ACK_OK;
        }

        case ROCKET_CMD_SET_MODE:
        {
            const uint8_t mode = payload[0];
            const uint16_t duration_s =
                RocketProtocol_ReadU16(payload + 1U);

            if (Behavior_RequestMode(
                    (BehaviorMode_t)mode,
                    (uint32_t)duration_s * 1000U,
                    now_ms) != BEHAVIOR_STATUS_OK)
            {
                return ROCKET_ACK_BAD_VALUE;
            }

            g_measurement_updates_enabled =
                (mode != ROCKET_MODE_STANDBY);

            *detail = mode;
            return ROCKET_ACK_OK;
        }

        case ROCKET_CMD_RETURN_STANDARD:
            Behavior_ReturnToStandard(now_ms);
            g_measurement_updates_enabled = true;
            return ROCKET_ACK_OK;

        case ROCKET_CMD_MANUAL_AIRBRAKE:
        {
            HAL_StatusTypeDef status;

            /*
             * A percentage command cannot replace an unfinished relative-step
             * movement. The caller must wait for the deferred motor ACK.
             */
            if (Airbrake_IsManualStepActive())
            {
                return ROCKET_ACK_BUSY;
            }

            /*
             * Sending an explicit position selects manual control.
             *
             * This intentionally re-enables the airbrake actuator even if it
             * was previously disabled. The command itself is an explicit
             * request to enter manual mode and move to the supplied position.
             */
            if (Behavior_SetControllerEnabled(false) !=
                BEHAVIOR_STATUS_OK)
            {
                return ROCKET_ACK_EXECUTION_ERROR;
            }

            g_airbrakes_enabled = true;
            Airbrake_SetManualOverride(true);

            status = Airbrake_SetTargetPercent(payload[0]);
            *detail = payload[0];

            if (status == HAL_BUSY)
            {
                return ROCKET_ACK_BUSY;
            }

            return (status == HAL_OK)
                ? ROCKET_ACK_OK
                : ROCKET_ACK_EXECUTION_ERROR;
        }

        case ROCKET_CMD_SET_SUBSYSTEM:
        {
            const uint8_t subsystem = payload[0];
            const bool enabled = (payload[1] != 0U);

            if (subsystem == ROCKET_SUBSYSTEM_FLIGHT_COMPUTER)
            {
                if (!enabled)
                {
                    HAL_StatusTypeDef retract_status;

                    /*
                     * Do not interrupt a relative motor movement. Its deferred
                     * completion ACK must occur before shutdown can continue.
                     */
                    if (Airbrake_IsManualStepActive())
                    {
                        return ROCKET_ACK_BUSY;
                    }

                    /*
                     * Disable the automatic controller. The unchanged
                     * Behavior_SetControllerEnabled(false) implementation also
                     * requests the safe 0% position.
                     */
                    if (Behavior_SetControllerEnabled(false) !=
                        BEHAVIOR_STATUS_OK)
                    {
                        return ROCKET_ACK_EXECUTION_ERROR;
                    }

                    /*
                     * Reissue the 0% request directly so this command can check
                     * whether the actuator accepted the safe target. The
                     * behavior function itself discards this HAL status.
                     */
                    retract_status = Airbrake_Retract();

                    if (retract_status == HAL_BUSY)
                    {
                        return ROCKET_ACK_BUSY;
                    }

                    if (retract_status != HAL_OK)
                    {
                        return ROCKET_ACK_EXECUTION_ERROR;
                    }

                    Airbrake_SetManualOverride(false);
                    g_airbrakes_enabled = false;
                }

                /*
                 * Normal behavior processing is suspended only after the safe
                 * retraction target has been accepted.
                 */
                g_flight_computer_enabled = enabled;

                *detail =
                    ((uint32_t)subsystem << 8) |
                    payload[1];

                return ROCKET_ACK_OK;
            }

            /*
             * Radio enable/disable remains private to radio_bridge.c.
             */
            return ROCKET_ACK_UNSUPPORTED;
        }
        case ROCKET_CMD_MOTOR_STEPS:
        {
            /*
             * A zero-length invocation is an internal completion poll from
             * RadioBridge_Task(). It is not a wire-format motor command.
             */
            if (payload_length == 0U)
            {
                HAL_StatusTypeDef update_status;
                Airbrake_Telemetry_t airbrake;

                /*
                 * Service the movement here so it continues even when Behavior
                 * updates are suspended by standby or flight-computer disable.
                 */
                update_status = Airbrake_Update(now_ms);
                airbrake = Airbrake_GetTelemetry();

                if (Airbrake_IsManualStepActive())
                {
                    return ROCKET_ACK_BUSY;
                }

                if ((airbrake.state == AIRBRAKE_STATE_FAULT) ||
                    (update_status == HAL_ERROR) ||
                    (update_status == HAL_TIMEOUT))
                {
                    *detail = airbrake.faults;
                    return ROCKET_ACK_EXECUTION_ERROR;
                }

                /*
                 * On success, return the physical final motor position.
                 */
                *detail = (uint32_t)airbrake.xactual;
                return ROCKET_ACK_OK;
            }


            if (payload_length != 2U)
            {
                return ROCKET_ACK_BAD_LENGTH;
            }

            /*
             * Relative motor movements are unavailable while the airbrake
             * subsystem is explicitly disabled.
             *
             * Unlike MANUAL_AIRBRAKE, MOTOR_STEPS is a low-level maintenance
             * operation and does not override the disabled state.
             */
            if (!g_airbrakes_enabled)
            {
                return ROCKET_ACK_BUSY;
            }

            {
                const int16_t steps =
                    RocketProtocol_ReadI16(payload);

                const HAL_StatusTypeDef status =
                    Airbrake_MoveRelativeSteps(steps);

                /*
                 * Preserve the signed requested step count in the ACK detail.
                 */
                *detail = (uint32_t)(int32_t)steps;

                if (status == HAL_BUSY)
                {
                    return ROCKET_ACK_BUSY;
                }

                return (status == HAL_OK)
                    ? ROCKET_ACK_OK
                    : ROCKET_ACK_EXECUTION_ERROR;
            }
        }

        case ROCKET_CMD_REQUEST_DIAGNOSTICS:
        {
            const uint8_t group = payload[0];
            const uint8_t value_index =
                (payload_length >= 2U) ? payload[1] : 0U;

            /*
             * The radio bridge handles radio diagnostics directly because the
             * counters and radio gates are private to radio_bridge.c.
             */
            if (group == ROCKET_DIAG_RADIO)
            {
                return ROCKET_ACK_UNSUPPORTED;
            }

            if (group == ROCKET_DIAG_SENSORS)
            {
                uint32_t failed_reads;

                if ((g_behavior_telemetry == NULL) ||
                    (value_index != 0U))
                {
                    return ROCKET_ACK_BAD_VALUE;
                }

                failed_reads = g_behavior_telemetry->failed_reads;
                if (failed_reads > 255U)
                {
                    failed_reads = 255U;
                }

                /*
                 * One compressed sensor ACK:
                 *
                 * [7:0]   IMU HAL status
                 * [15:8]  magnetometer HAL status
                 * [23:16] barometer HAL status
                 * [31:24] failed-read count, saturated at 255
                 */
                *detail =
                    ((uint32_t)(uint8_t)
                        g_behavior_telemetry->imu_status) |
                    ((uint32_t)(uint8_t)
                        g_behavior_telemetry->mag_status << 8) |
                    ((uint32_t)(uint8_t)
                        g_behavior_telemetry->baro_status << 16) |
                    (failed_reads << 24);

                return ROCKET_ACK_OK;
            }

            if (group == ROCKET_DIAG_ESTIMATOR)
            {
                int32_t first;
                int32_t second;

                if (g_behavior_telemetry == NULL)
                {
                    return ROCKET_ACK_EXECUTION_ERROR;
                }

                switch (value_index)
                {
                    case 0U:
                        /*
                         * [15:0]  altitude in 0.1 m, signed
                         * [31:16] velocity in 0.01 m/s, signed
                         */
                        if (g_behavior_telemetry->ekf_data_valid)
                        {
                            first = (int32_t)
                                (g_behavior_telemetry->ekf_altitude_m *
                                 10.0f);
                            second = (int32_t)
                                (g_behavior_telemetry->ekf_velocity_m_s *
                                 100.0f);
                        }
                        else
                        {
                            first = 0;
                            second = 0;
                        }

                        if (first > 32767)  { first = 32767; }
                        if (first < -32768) { first = -32768; }
                        if (second > 32767)  { second = 32767; }
                        if (second < -32768) { second = -32768; }

                        *detail =
                            (uint32_t)(uint16_t)(int16_t)first |
                            ((uint32_t)(uint16_t)(int16_t)second << 16);

                        return ROCKET_ACK_OK;

                    case 1U:
                        /*
                         * [15:0]  acceleration in 0.01 m/s^2, signed
                         * [31:16] predicted apogee in 0.1 m, unsigned
                         */
                        if (g_behavior_telemetry->ekf_data_valid)
                        {
                            first = (int32_t)
                                (g_behavior_telemetry->
                                    ekf_acceleration_m_s2 * 100.0f);
                        }
                        else
                        {
                            first = 0;
                        }

                        if (g_behavior_telemetry->
                            controller_output_valid)
                        {
                            second = (int32_t)
                                (g_behavior_telemetry->
                                    predicted_apogee_m * 10.0f);
                        }
                        else
                        {
                            second = 0;
                        }

                        if (first > 32767)  { first = 32767; }
                        if (first < -32768) { first = -32768; }
                        if (second > 65535) { second = 65535; }
                        if (second < 0)     { second = 0; }

                        *detail =
                            (uint32_t)(uint16_t)(int16_t)first |
                            ((uint32_t)(uint16_t)second << 16);

                        return ROCKET_ACK_OK;

                    case 2U:
                    {
                        uint32_t target_dm;
                        uint32_t flags = 0U;

                        /*
                         * [15:0]  target apogee in 0.1 m
                         * [22:16] requested deployment percent
                         * [27:23] validity/control flags
                         */
                        first = (int32_t)
                            (g_behavior_telemetry->
                                target_apogee_m * 10.0f);

                        if (first < 0)
                        {
                            target_dm = 0U;
                        }
                        else if (first > 65535)
                        {
                            target_dm = 65535U;
                        }
                        else
                        {
                            target_dm = (uint32_t)first;
                        }

                        if (g_behavior_telemetry->ekf_data_valid)
                        {
                            flags |= 0x01U;
                        }
                        if (g_behavior_telemetry->fusion_data_valid)
                        {
                            flags |= 0x02U;
                        }
                        if (g_behavior_telemetry->controller_enabled)
                        {
                            flags |= 0x04U;
                        }
                        if (g_behavior_telemetry->controller_active)
                        {
                            flags |= 0x08U;
                        }
                        if (g_behavior_telemetry->
                            controller_output_valid)
                        {
                            flags |= 0x10U;
                        }

                        *detail =
                            target_dm |
                            ((uint32_t)
                                (g_behavior_telemetry->
                                    controller_requested_percent &
                                 0x7FU) << 16) |
                            ((flags & 0x1FU) << 23);

                        return ROCKET_ACK_OK;
                    }

                    default:
                        return ROCKET_ACK_BAD_VALUE;
                }
            }

            if (group == ROCKET_DIAG_AIRBRAKE)
            {
                const Airbrake_Telemetry_t airbrake =
                    Airbrake_GetTelemetry();

                switch (value_index)
                {
                    case 0U:
                        *detail = (uint32_t)airbrake.xactual;
                        return ROCKET_ACK_OK;

                    case 1U:
                        *detail = (uint32_t)airbrake.target_counts;
                        return ROCKET_ACK_OK;

                    case 2U:
                        *detail = airbrake.faults;
                        return ROCKET_ACK_OK;

                    case 3U:
                        *detail = airbrake.last_drv_status;
                        return ROCKET_ACK_OK;

                    case 4U:
                        *detail = airbrake.last_gstat;
                        return ROCKET_ACK_OK;

                    case 5U:
                        /*
                         * bits 0-3:   Airbrake_State_t
                         * bits 4-10:  current percentage
                         * bits 11-17: target percentage
                         * bit 18:     calibrated
                         * bit 19:     deployment allowed
                         * bit 20:     manual override
                         * bit 21:     thermal limited
                         * bit 22:     manual-step ownership
                         * bit 23:     actuator moving
                         */
                        *detail =
                            ((uint32_t)airbrake.state & 0x0FU) |
                            ((uint32_t)
                                (airbrake.current_percent & 0x7FU) << 4) |
                            ((uint32_t)
                                (airbrake.target_percent & 0x7FU) << 11) |
                            (airbrake.calibrated ? (1UL << 18) : 0U) |
                            (airbrake.deployment_allowed
                                ? (1UL << 19) : 0U) |
                            (airbrake.manual_override
                                ? (1UL << 20) : 0U) |
                            (airbrake.thermal_limited
                                ? (1UL << 21) : 0U) |
                            (Airbrake_IsManualStepActive()
                                ? (1UL << 22) : 0U) |
                            (Airbrake_IsBusy()
                                ? (1UL << 23) : 0U);

                        return ROCKET_ACK_OK;

                    default:
                        return ROCKET_ACK_BAD_VALUE;
                }
            }

            if (group == ROCKET_DIAG_STORAGE)
            {
                /*
                 * STORAGE IMPLEMENTATION GOES HERE.
                 *
                 * When storage exists, value_index 0 can return a compressed
                 * status word such as:
                 *
                 * bits 0-7:   storage state
                 * bits 8-15:  last error
                 * bits 16-31: sectors or records used
                 */
                *detail = 0U;
                return ROCKET_ACK_UNSUPPORTED;
            }

            return ROCKET_ACK_BAD_VALUE;
        }

        case ROCKET_CMD_CLEAR_FAULTS:
        {
            const HAL_StatusTypeDef status =
                Airbrake_ClearFaults();
            const Airbrake_Telemetry_t airbrake =
                Airbrake_GetTelemetry();

            /*
             * Report any fault bits that remain after the attempt.
             */
            *detail = airbrake.faults;

            return ((status == HAL_OK) &&
                    (airbrake.faults == AIRBRAKE_FAULT_NONE))
                ? ROCKET_ACK_OK
                : ROCKET_ACK_EXECUTION_ERROR;
        }

        default:
            return ROCKET_ACK_UNSUPPORTED;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_USB_PCD_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USBX_Init();
  /* USER CODE BEGIN 2 */

  /*
   * Initialize the AMBAR-to-CDC adapter before allowing the host
   * to activate the CDC interface.
   */
  if (!USBComm_Init())
  {
      Error_Handler();
  }

  /*
   * Connect the device electrically to the USB host.
   */
  if (HAL_PCD_Start(&hpcd_USB_DRD_FS) != HAL_OK)
  {
      Error_Handler();
  }

  /*
     * SPI3 and GPIOD pin 2 are the fitted W25Q64JV interface. A storage failure
     * is recorded for diagnostics but does not stop estimator, radio, USB, or
     * airbrake operation; flight safety must not depend on the logger.
     */
    W25Q64_Attach(&g_memory_flash,
                  &hspi3,
                  SPI3_CS_GPIO_Port,
                  SPI3_CS_Pin);

    g_memory_flash_init_result = W25Q64_Init(&g_memory_flash);
    if (g_memory_flash_init_result == W25Q64_OK)
    {
        g_memory_log_result = W25Q64_LogOpen(&g_memory_flash);
        g_memory_log_records = W25Q64_LogGetCount();
        g_memory_log_capacity = W25Q64_LogGetCapacity();
        g_memory_log_corrupt_records = W25Q64_LogGetCorruptCount();
    }

  // init radio
  if (RadioBridge_Init() != HAL_OK)
  {
      Error_Handler();
  }
  // init behavior (all testing/procedure. Keeps main simple and empty)
  Behavior_DefaultConfig(&g_behavior_config);

  // Preflight values: edit these directly or through the debugger.
  g_behavior_config.target_apogee_m = 970.0f; // about 3000 ft.
  g_behavior_config.initial_altitude_agl_m = 0.0f; // measured from baro
  g_behavior_config.launch_site_altitude_msl_m = 0.0f; // physical setup
  g_behavior_config.deployment_min = 0.0f;
  g_behavior_config.deployment_max = 1.0f;
  g_behavior_config.altitude_tolerance_m = 9.144f; // 30 ft
  g_behavior_config.controller_enabled = true;


  // set board mounting
  g_behavior_config.imu_alignment = FusionRemapAlignmentPXPYPZ;
  g_behavior_config.mag_alignment = FusionRemapAlignmentPXPYPZ;

  if (Behavior_Init(&g_behavior_config, HAL_GetTick()) != HAL_OK)
  {
      // maybe run an errpor? idk what to put here
  }

  g_behavior_telemetry = Behavior_GetTelemetry();

  // set airbrake settings
  Airbrake_Config_t airbrake_cfg;

  Airbrake_DefaultConfig(&airbrake_cfg);

  /* Mechanical calibration */
  airbrake_cfg.closed_counts = 0;        // 0% deployment
  airbrake_cfg.open_counts   = 200 * 256 * 3;    // 100% deployment; replace with measured value

  /*motor settings */
  airbrake_cfg.ihold_run  = 16;
  airbrake_cfg.irun_run   = 31;
  airbrake_cfg.scaler_run = 0;

  airbrake_cfg.amax_normal = 20000;
  airbrake_cfg.vmax_normal = 200000;

  airbrake_cfg.move_timeout_ms = 7000U;

  // the airbrake determines if it can deploy
  airbrake_cfg.launch_accel_g = 3.0f;
  airbrake_cfg.burnout_accel_g = 1.5f;
  airbrake_cfg.burnout_drop_g = 2.0f;
  airbrake_cfg.post_burnout_delay_ms = 1000U; // 1 second

  // initialize
  if (Airbrake_Init(&airbrake_cfg) != HAL_OK)
  {
	Error_Handler();
  }

  // change the define to anything else if you want to be able to control the airbrake outside of burnout detection.
  Airbrake_SetManualOverride(APP_ENABLE_AIRBRAKE_BENCH_OVERRIDE != 0U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // basically needed for anything regarding timing
		const uint32_t now_ms = HAL_GetTick();

		/*
		 * Track whether Behavior_Update() serviced the actuator during this
		 * iteration. During standby or flight-computer disable, the airbrake
		 * still requires periodic supervision while returning to 0%.
		 */
		bool behavior_updated = false;

		/*
		 * Flight-computer disable suspends all normal application logic. The
		 * radio task remains outside this gate so commands can re-enable it.
		 */
		if (g_flight_computer_enabled)
		{
			// change to true in debug menu if you have a new config you want to apply
			if (g_apply_behavior_config)
			{
				g_apply_behavior_config = false;
				(void)Behavior_ApplyConfig(&g_behavior_config, now_ms);
			}

			// was for the comprehensive ekf->motor test, this can be removed if you don't want it
			if (g_request_comprehensive_test)
			{
				g_request_comprehensive_test = false;
				(void)Behavior_RequestMode(
					BEHAVIOR_MODE_TEST_FULL_PIPELINE,
					0U,
					now_ms);
			}

			// change in debug menu to return to standard mode.
			if (g_request_standard_mode)
			{
				g_request_standard_mode = false;
				Behavior_ReturnToStandard(now_ms);
				g_measurement_updates_enabled = true;
			}

			/* Standby skips new Behavior measurements/estimation/control while
			 * still allowing configuration and command processing. */
			if (g_measurement_updates_enabled)
			{
				Behavior_Update(now_ms);
				behavior_updated = true;
			}
		}

		/*
		 * Behavior_Update() normally calls Airbrake_Update(). When behavior is
		 * suspended, continue servicing motor feedback, timeout detection,
		 * thermal limiting, and completion of the safe retraction.
		 */
		if (!behavior_updated)
		{
			(void)Airbrake_Update(now_ms);
		}

		/* Command interpretation and acknowledgements are never suspended. */
		RadioBridge_Task(g_behavior_telemetry, now_ms);

		/* Archive the stable Rocket Protocol payload, not compiler-dependent RAM. */
		Memory_LogTask(g_behavior_telemetry, now_ms, behavior_updated);

		/* A requested full erase waits here until measurement updates are stopped. */
		Memory_ServiceEraseRequest();

		// tons of test and debug displays
  //      float acc = g_behavior_telemetry->ekf_acceleration_m_s2;
  //      float alt = g_behavior_telemetry->ekf_altitude_m;
  //      float vel = g_behavior_telemetry->ekf_velocity_m_s;
  //
  //      printf("Acceleration: %f \n", acc);
  //      printf("Altitude: %f \n", alt);
  //      printf("Velocity: %f \n", vel);
  //
  //      RocketSensorRawData_t rawd = g_behavior_telemetry->raw;
  //      printf("imu Raw value x gyro: %f \n", rawd.imu[0]);
  //      printf("imu Raw value y gyro: %f \n",rawd.imu[1]);
  //      printf("imu Raw value y gyro: %f \n", rawd.imu[2]);
  //      printf("imu Raw value x: %f g \n", rawd.imu[3]);
  //      printf("imu Raw value y: %f g \n", rawd.imu[4]);
  //      printf("imu Raw value z: %f g \n", rawd.imu[5]);
  //      //printf("baro Raw value 1: %f \n", rawd.baro[0]);
  //      //printf("baro Raw value 2: %f \n", rawd.baro[1]);
  //      // mag y+ == z+
  //      //
  //      printf("mag Raw value x: %f \n", rawd.mag[0]);
  //      printf("mag Raw value y: %f \n", rawd.mag[2]);
  //      printf("mag Raw value z: %f \n", rawd.mag[0]);

  //      position.altitude_agl = openrocket_run_samples[step].altitude_m;
  //      position.vertical_acceleration = openrocket_run_samples[step].acceleration_mps2;
  //      position.vertical_velocity = openrocket_run_samples[step].velocity_mps;
  //
  //      float deploy = Controller_NewDeployment(&cont, position, 1.0/100);
  //
  //      step ++;
  //
  //      printf("deployment: %f \n",deploy);
  //
  //      Airbrake_Update(now_ms);
  //      Airbrake_SetTargetPercent((uint8_t) (deploy * 100));

		// delay is needed for tests, but comment out
		//HAL_Delay(10u);
		// airbrake actuation happens inside behavior

		/*
		 * USBX standalone-mode scheduler.
		 *
		 * This must run continuously so that USB enumeration,
		 * CDC transfers, and USBX class processing can progress.
		 */
		ux_system_tasks_run();
		/*
		 * CDC transfers, AMBAR decoding, and PING ACK generation.
		 */
		USBComm_Task();

		/*
		 * Replay flash only after an accepted EXPORT_LOG USB command.
		 * Physical connection and PING alone never start archive transmission.
		 */
		Memory_ExportTask();

		// don't touch anything else, it's auto-generated
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIGITAL;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the CRS APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60808CD3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60808CD3;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x60808CD3;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */
  /*
   * USB_DRD_FS packet-memory allocation.
   *
   * dev_endpoints is 8, so the PMA buffer area begins after the
   * eight-entry endpoint buffer table.
   */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS,
                      0x00U,
                      PCD_SNG_BUF,
                      0x20U);  /* EP0 OUT, 64 bytes */

  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS,
                      0x80U,
                      PCD_SNG_BUF,
                      0x60U);  /* EP0 IN, 64 bytes */

  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS,
                      0x81U,
                      PCD_SNG_BUF,
                      0xA0U);  /* CDC bulk IN */

  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS,
                      0x01U,
                      PCD_SNG_BUF,
                      0xE0U);  /* CDC bulk OUT */

  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS,
                      0x82U,
                      PCD_SNG_BUF,
                      0x120U); /* CDC notification IN */
  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_4_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_NRESET_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_CS_Pin|MOTOR_DRV_ENN_Pin|MOTOR_SLEEPN_Pin|LED_6_Pin
                          |LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_4_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_4_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_1_Pin LORA_NRESET_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_1_Pin|LORA_NRESET_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO1_Pin MOTOR_DIAG0_Pin I2C3_INT_Pin */
  GPIO_InitStruct.Pin = LORA_DIO1_Pin|MOTOR_DIAG0_Pin|I2C3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_CS_Pin MOTOR_DRV_ENN_Pin MOTOR_SLEEPN_Pin LED_6_Pin
                           LED_5_Pin */
  GPIO_InitStruct.Pin = MOTOR_CS_Pin|MOTOR_DRV_ENN_Pin|MOTOR_SLEEPN_Pin|LED_6_Pin
                          |LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_INT_Pin MOTOR_DIAG1_Pin I2C1_INT_Pin */
  GPIO_InitStruct.Pin = I2C2_INT_Pin|MOTOR_DIAG1_Pin|I2C1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_SENSE_Pin */
  GPIO_InitStruct.Pin = USB_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    ITM_SendChar((*ptr++));
  }
  return len;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
