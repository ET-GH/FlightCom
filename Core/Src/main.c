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
//#include "controller.h"
//#include "usb_comm.h"
//#include "altitude_ekf.h"
//#include "Fusion.h"
#include "behavior.h"
// only for one test
//#include "openrocket_run_data.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ENABLE_AIRBRAKE_BENCH_OVERRIDE  0U
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
      HAL_Delay(10u);
      // airbrake actuation happens inside behavior

      // don't touch anything else, it's auto-generated

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
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
