#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32h5xx_hal.h"
#include "altitude_ekf.h"
#include "rocket_sensors.h"
#include "Fusion.h"

/*
 * High-level operating modes.
 *
 * STANDARD:
 *   Real sensors -> Fusion -> altitude EKF -> controller percentage.
 *   Physical airbrake and motor calls remain disabled in behavior.c.
 *
 * TEST_RAW_SENSORS:
 *   Reads and converts sensors only.
 *
 * TEST_SYNTHETIC_EKF:
 *   Synthetic vertical flight -> EKF -> controller -> simulated drag.
 *   Fusion is intentionally bypassed so the EKF/controller can be isolated.
 *
 * TEST_FUSION:
 *   Real IMU/magnetometer -> Fusion only.
 *
 * TEST_REAL_IMU_EKF:
 *   Real sensors -> Fusion -> EKF -> controller percentage.
 *   This is similar to STANDARD but remains a separately selectable diagnostic mode.
 *
 * TEST_FULL_PIPELINE:
 *   Consistent synthetic body-frame IMU/magnetometer -> Fusion -> EKF ->
 *   controller -> simulated airbrake drag.  This is the comprehensive test.
 */
typedef enum
{
    BEHAVIOR_MODE_STANDARD = 0,
    BEHAVIOR_MODE_TEST_RAW_SENSORS,
    BEHAVIOR_MODE_TEST_SYNTHETIC_EKF,
    BEHAVIOR_MODE_TEST_FUSION,
    BEHAVIOR_MODE_TEST_REAL_IMU_EKF,
    BEHAVIOR_MODE_TEST_FULL_PIPELINE
} BehaviorMode_t;

typedef enum
{
    BEHAVIOR_STATUS_OK = 0,
    BEHAVIOR_STATUS_SENSOR_INIT_FAILED,
    BEHAVIOR_STATUS_SENSOR_READ_FAILED,
    BEHAVIOR_STATUS_UNSUPPORTED_MODE,
    BEHAVIOR_STATUS_BAD_ARGUMENT,
    BEHAVIOR_STATUS_INVALID_CONFIG
} BehaviorStatus_t;

typedef enum
{
    BEHAVIOR_FLIGHT_PHASE_PAD = 0,
    BEHAVIOR_FLIGHT_PHASE_POWERED_ASCENT,
    BEHAVIOR_FLIGHT_PHASE_COAST,
    BEHAVIOR_FLIGHT_PHASE_DESCENT,
    BEHAVIOR_FLIGHT_PHASE_LANDED
} BehaviorFlightPhase_t;

/*
 * Preflight/standard-mode configuration.
 *
 * All altitude quantities are meters.  target_apogee_m is AGL.
 * deployment_min/max are normalized controller commands in [0, 1].
 */
typedef struct
{
    float target_apogee_m;
    float initial_altitude_agl_m;
    float launch_site_altitude_msl_m;

    float deployment_min;
    float deployment_max;
    bool controller_enabled;

    float estimator_rate_hz;
    FusionConvention fusion_convention;
    FusionRemapAlignment imu_alignment;
    FusionRemapAlignment mag_alignment;

    float fusion_gain;
    float gyroscope_range_dps;
    float acceleration_rejection_deg;
    float magnetic_rejection_deg;
    float fusion_recovery_period_s;

    /* Raw sensor conversion/calibration values. */
    float gyro_dps_per_lsb;
    float accel_g_per_lsb;
    FusionVector gyro_offset_dps;
    FusionVector accel_offset_g;
    FusionVector mag_hard_iron_offset;
    FusionMatrix mag_soft_iron_matrix;

    /* Pass/fail requirements. */
    float altitude_tolerance_m;
    float fusion_roll_tolerance_deg;
    float fusion_pitch_tolerance_deg;
    float fusion_yaw_tolerance_deg;

    /* Comprehensive synthetic-test model. */
    float synthetic_pad_warmup_s;
    float synthetic_burn_time_s;
    float synthetic_powered_accel_m_s2;
    float synthetic_base_drag_k;
    float synthetic_airbrake_drag_k;
} BehaviorConfig_t;

typedef struct
{
    BehaviorMode_t mode;
    BehaviorMode_t previous_mode;
    BehaviorStatus_t status;
    BehaviorFlightPhase_t flight_phase;

    bool initialized;
    bool mode_changed;
    bool auto_return_enabled;

    uint32_t mode_entered_ms;
    uint32_t auto_return_after_ms;

    float imu_samples;
    float mag_samples;
    float baro_samples;
    uint32_t failed_reads;
    uint32_t synthetic_samples;
    uint32_t full_pipeline_samples;

    /* Active configuration snapshot for easy debugger access. */
    float target_apogee_m;
    float initial_altitude_agl_m;
    float launch_site_altitude_msl_m;
    float deployment_min;
    float deployment_max;
    float altitude_tolerance_m;
    bool controller_enabled;

    RocketSensorRawData_t raw;
    HAL_StatusTypeDef imu_status;
    HAL_StatusTypeDef mag_status;
    HAL_StatusTypeDef baro_status;

    /* Converted/calibrated real or synthetic body-frame sensors. */
    float gyro_dps[3];
    float accel_g[3];
    float mag[3];

    /* Fusion output and diagnostics. */
    float fusion_roll_deg;
    float fusion_pitch_deg;
    float fusion_yaw_deg;
    float fusion_earth_accel_g[3];
    float vertical_acceleration_m_s2;

    float fusion_acceleration_error_deg;
    float fusion_magnetic_error_deg;
    bool fusion_accelerometer_ignored;
    bool fusion_magnetometer_ignored;
    bool fusion_startup;

    /* Synthetic truth values used by test modes. */
    float test_time_s;
    float test_flight_time_s;
    float test_true_altitude_m;
    float test_true_velocity_m_s;
    float test_true_acceleration_m_s2;
    float test_roll_deg;
    float test_pitch_deg;
    float test_yaw_deg;
    float test_gyro_dps[3];
    float test_accel_g[3];
    float test_mag[3];

    /* Fusion accuracy in the full-pipeline synthetic test. */
    float fusion_roll_error_deg;
    float fusion_pitch_error_deg;
    float fusion_yaw_error_deg;
    float max_fusion_roll_error_deg;
    float max_fusion_pitch_error_deg;
    float max_fusion_yaw_error_deg;

    /* EKF output and accuracy. */
    float ekf_altitude_m;
    float ekf_velocity_m_s;
    float ekf_acceleration_m_s2;
    float ekf_bias_m_s2;
    float altitude_error_m;
    float velocity_error_m_s;
    float max_altitude_error_m;
    float max_velocity_error_m_s;

    /* Barometer correction supplied by compensated BMP388 code. */
    float barometer_altitude_agl_m;
    uint32_t barometer_altitude_timestamp_ms;
    bool barometer_altitude_valid;
    bool barometer_correction_used;

    /* Controller output only.  No motor command is sent. */
    float predicted_apogee_m;
    float controller_raw_deployment;
    float controller_requested_deployment;
    uint8_t controller_requested_percent;
    bool controller_active;

    /* Final comprehensive-test results. */
    float true_apogee_m;
    float estimated_apogee_m;
    float true_apogee_error_m;
    bool apogee_reached;
    bool fusion_data_valid;
    bool fusion_within_tolerance;
    bool ekf_data_valid;
    bool ekf_within_altitude_tolerance;
    bool controller_output_valid;
    bool controller_apogee_within_tolerance;
    bool full_pipeline_complete;
    bool full_pipeline_pass;
} BehaviorTelemetry_t;

/* Fill a configuration structure with safe/default values. */
void Behavior_DefaultConfig(BehaviorConfig_t *config);

/*
 * Initialize sensors, Fusion, EKF, controller, and the state machine.
 * Pass NULL to use Behavior_DefaultConfig values.
 */
HAL_StatusTypeDef Behavior_Init(const BehaviorConfig_t *config,
                                uint32_t now_ms);

/* Apply updated preflight values and reset Fusion/EKF/controller state. */
BehaviorStatus_t Behavior_ApplyConfig(const BehaviorConfig_t *config,
                                      uint32_t now_ms);

/* Convenience preflight setters. */
BehaviorStatus_t Behavior_SetTargetApogee(float target_apogee_m,
                                          uint32_t now_ms);
BehaviorStatus_t Behavior_SetInitialAltitudeAGL(float initial_altitude_agl_m,
                                                uint32_t now_ms);
BehaviorStatus_t Behavior_SetDeploymentBounds(float deployment_min,
                                              float deployment_max);
BehaviorStatus_t Behavior_SetControllerEnabled(bool enabled);

/*
 * Submit a compensated barometric altitude reading.
 * The current BMP388 driver only exposes raw ADC values, so another layer must
 * perform Bosch compensation and then call one of these functions.
 */
BehaviorStatus_t Behavior_SubmitBarometerAltitudeAGL(float altitude_agl_m,
                                                     uint32_t now_ms);
BehaviorStatus_t Behavior_SubmitBarometerAltitudeMSL(float altitude_msl_m,
                                                     uint32_t now_ms);

/* Run one non-blocking iteration. Call continuously from while (1). */
void Behavior_Update(uint32_t now_ms);

/* Enter a mode. duration_ms == 0 keeps it active until another request. */
BehaviorStatus_t Behavior_RequestMode(BehaviorMode_t mode,
                                      uint32_t duration_ms,
                                      uint32_t now_ms);

void Behavior_ReturnToStandard(uint32_t now_ms);

BehaviorMode_t Behavior_GetMode(void);
const BehaviorConfig_t *Behavior_GetConfig(void);
const volatile BehaviorTelemetry_t *Behavior_GetTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* BEHAVIOR_H */
