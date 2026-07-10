#include "behavior.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "controller.h"

/*
 * Physical actuation is intentionally disabled for now
 */
#include "airbrake.h"
#include "tmc5240.h"

#define BEHAVIOR_GRAVITY_M_S2               9.80665f
#define BEHAVIOR_MIN_DT_S                    0.001f
#define BEHAVIOR_MAX_DT_S                    0.050f
#define BEHAVIOR_BARO_STALE_MS               250U
#define BEHAVIOR_SYNTHETIC_BARO_DIVIDER      5U

/* Data-ready bits used to guarantee that every EKF cycle uses a fresh sample
 * from every real sensor.  The BMP388 is the slowest configured sensor at
 * 50 Hz, so its data-ready flag naturally limits the real EKF to at most 50 Hz.
 */
#define BEHAVIOR_IMU_READY_MASK               0x03U /* XLDA | GDA */
#define BEHAVIOR_MAG_READY_MASK               0x08U /* ZYXDA */
#define BEHAVIOR_BARO_READY_MASK              0x60U /* drdy_press | drdy_temp */

#define BEHAVIOR_BARO_REFERENCE_SAMPLES       50U
#define BEHAVIOR_BARO_REFERENCE_DELAY_MS      20U

#define BEHAVIOR_EKF_VAR_BARO                2.25f
#define BEHAVIOR_EKF_VAR_IMU_ACCEL           0.16f
#define BEHAVIOR_EKF_VAR_PROCESS_JERK       25.0f
#define BEHAVIOR_EKF_VAR_PROCESS_BIAS        0.0001f

#define BEHAVIOR_SYNTH_ACCEL_BIAS_M_S2       0.20f
#define BEHAVIOR_SYNTH_GYRO_BIAS_DPS         0.40f
#define BEHAVIOR_SYNTH_ACCEL_NOISE_M_S2      0.20f
#define BEHAVIOR_SYNTH_GYRO_NOISE_DPS        0.08f
#define BEHAVIOR_SYNTH_MAG_NOISE              0.005f
#define BEHAVIOR_SYNTH_BARO_NOISE_M           1.50f

// logic classes
static AltitudeEKF_t altitude_ekf;
static Controller controller;
static FusionAhrs fusion_ahrs;
static FusionBias fusion_bias;

// data
static BehaviorConfig_t behavior_config;
static volatile BehaviorTelemetry_t telemetry;

// timing
/* last_sensor_attempt_ms controls status polling.  last_sensor_update_ms is
 * advanced only after one complete synchronized IMU + magnetometer +
 * barometer sample has been acquired successfully.
 */
static uint32_t last_sensor_attempt_ms;
static uint32_t last_sensor_update_ms;
static uint32_t last_synthetic_update_ms;
static uint32_t synthetic_baro_counter;
static float synthetic_ekf_elapsed_s;
static float synthetic_accel_sum_m_s2;
static uint32_t synthetic_accel_count;
static uint32_t noise_state;

static FusionVector latest_magnetometer;
static bool latest_magnetometer_valid;

static float submitted_baro_altitude_agl_m;
static uint32_t submitted_baro_timestamp_ms;
static uint32_t submitted_baro_sequence;
static uint32_t consumed_baro_sequence;

static float synthetic_time_s;
static float synthetic_previous_velocity_m_s;
static FusionQuaternion synthetic_truth_quaternion;

// change between a testing or standard mode
static void Behavior_EnterMode(BehaviorMode_t mode,
                               uint32_t duration_ms,
                               uint32_t now_ms);
// verify mode
static bool Behavior_IsSupportedMode(BehaviorMode_t mode);
// verify configuration data
static bool Behavior_ValidateConfig(const BehaviorConfig_t *config);
// transfer config data to the main data struct
static void Behavior_CopyConfigToTelemetry(void);
// reset/init fusion
static void Behavior_ResetFusion(void);
// reset/init controller
static void Behavior_ResetEstimatorController(uint32_t now_ms);
// reset synthetic data to 0
static void Behavior_ResetSyntheticTest(uint32_t now_ms,
                                        bool reset_fusion);
// update logic for each behavior
static void Behavior_UpdateStandard(uint32_t now_ms);
static void Behavior_UpdateRawSensorTest(uint32_t now_ms);
static void Behavior_UpdateSyntheticEkfTest(uint32_t now_ms);
static void Behavior_UpdateFusionTest(uint32_t now_ms);
static void Behavior_UpdateRealImuEkfTest(uint32_t now_ms);
static void Behavior_UpdateFullPipelineTest(uint32_t now_ms);

// collect samples and measure period
static bool Behavior_ReadAndConvertSensors(uint32_t now_ms,
                                           float *dt_s);
// raw through fusion vectors
static void Behavior_ConvertRawSensors(FusionVector *gyroscope,
                                       FusionVector *accelerometer,
                                       FusionVector *magnetometer);
// fusion logic
static void Behavior_UpdateFusion(const FusionVector gyroscope,
                                  const FusionVector accelerometer,
                                  const FusionVector magnetometer,
                                  bool magnetometer_valid,
                                  float dt_s);
// pipeline logic
static void Behavior_UpdateRealPipeline(uint32_t now_ms,
                                        bool update_ekf,
                                        bool update_controller);
// pass baro through ekf
static void Behavior_ConsumeBarometerCorrection(uint32_t now_ms);
// pass data through controller
static void Behavior_UpdateController(uint32_t now_ms, float dt_s,
                                      bool allow_deployment);
// push data from ekf to telemetry struct
static void Behavior_CopyEkfTelemetry(void);
// push data from fusion to telemetry struct
static void Behavior_CopyFusionTelemetry(const FusionVector earth_acceleration);
// increment synthetic data and flight conditions
static void Behavior_AdvanceSyntheticDynamics(float dt_s);
// return fusion data of rocket
static FusionVector Behavior_GetSyntheticBodyRate(void);
// fusion quaternion don't make sense to me but it seems important, I didn't bother trying to understand (thank you ai)
static void Behavior_UpdateTruthQuaternion(FusionVector body_rate_dps,
                                           float dt_s);
// align vectors and reference
static FusionVector Behavior_RotateEarthToBody(FusionVector earth_vector,
                                               FusionQuaternion body_to_earth);
// simulate sensors w/ noise
static void Behavior_CreateSyntheticSensors(FusionVector body_rate_truth,
                                            FusionVector *gyroscope,
                                            FusionVector *accelerometer,
                                            FusionVector *magnetometer);
// update entire telemetry struct
static void Behavior_UpdateSyntheticTruthTelemetry(FusionVector gyroscope,
                                                   FusionVector accelerometer,
                                                   FusionVector magnetometer);
// update errors and the like
static void Behavior_UpdateTestMetrics(void);
// verify test results
static void Behavior_FinalizeApogeeResult(void);
// status update on estimator
static float Behavior_GetUpdatePeriodSeconds(void);
static uint32_t Behavior_GetUpdatePeriodMilliseconds(void);
// math helpers
static float Behavior_ClampFloat(float value,
                                 float minimum,
                                 float maximum);
static float Behavior_Absolute(float value);
static float Behavior_AngleDifferenceDegrees(float estimate,
                                             float truth);
static float Behavior_Noise(float amplitude);
static bool Behavior_Finite(float value);
// vector alignment again
static float Behavior_EarthVerticalToPositiveUp(FusionVector earth_acceleration);

// configuration of behavior and default values
void Behavior_DefaultConfig(BehaviorConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    // allocate space
    memset(config, 0, sizeof(*config));

    // replace default values
    config->target_apogee_m = 100.0f;
    config->initial_altitude_agl_m = 0.0f;
    config->launch_site_altitude_msl_m = 0.0f;

    config->deployment_min = 0.0f;
    config->deployment_max = 1.0f;
    config->controller_enabled = true;

    config->estimator_rate_hz = 100.0f;
    config->fusion_convention = FusionConventionNed;
    config->imu_alignment = FusionRemapAlignmentPXPYPZ;
    config->mag_alignment = FusionRemapAlignmentPXPYPZ;

    config->fusion_gain = 0.5f;
    config->gyroscope_range_dps = 4000.0f;
    config->acceleration_rejection_deg = 10.0f;
    config->magnetic_rejection_deg = 10.0f;
    config->fusion_recovery_period_s = 5.0f;

    /* Initial conversion values for the current +/-4000 dps, +/-32 g setup. */
    config->gyro_dps_per_lsb = 4000.0f / 32768.0f;
    config->accel_g_per_lsb = 32.0f / 32768.0f;
    config->gyro_offset_dps = FUSION_VECTOR_ZERO;
    config->accel_offset_g = FUSION_VECTOR_ZERO;
    config->mag_hard_iron_offset = FUSION_VECTOR_ZERO;
    config->mag_soft_iron_matrix = FUSION_MATRIX_IDENTITY;

    config->altitude_tolerance_m = 9.144f; /* 30 ft  because I thought that was fair*/
    config->fusion_roll_tolerance_deg = 10.0f;
    config->fusion_pitch_tolerance_deg = 10.0f;
    config->fusion_yaw_tolerance_deg = 15.0f;

    config->synthetic_pad_warmup_s = 3.5f;
    config->synthetic_burn_time_s = 2.0f;
    config->synthetic_powered_accel_m_s2 = 20.0f;
    config->synthetic_base_drag_k = 0.0010f;
    config->synthetic_airbrake_drag_k = 0.0040f;
}

// initialize behavior
HAL_StatusTypeDef Behavior_Init(const BehaviorConfig_t *config,
                                uint32_t now_ms)
{
    HAL_StatusTypeDef sensor_status;
    BehaviorConfig_t defaults;
    uint32_t init_complete_ms;

    Behavior_DefaultConfig(&defaults);

    if ((config != NULL) && Behavior_ValidateConfig(config))
    {
        behavior_config = *config;
    }
    else
    {
        behavior_config = defaults;
    }

    // allocate telemetry
    memset((void *)&telemetry, 0, sizeof(telemetry));
    latest_magnetometer = FUSION_VECTOR_ZERO;
    latest_magnetometer_valid = false;

    submitted_baro_altitude_agl_m = behavior_config.initial_altitude_agl_m;
    submitted_baro_timestamp_ms = now_ms;
    submitted_baro_sequence = 0U;
    consumed_baro_sequence = 0U;

    noise_state = 0x13579BDFU;

    sensor_status = RocketSensors_Init();
    if (sensor_status == HAL_OK)
    {
        /* Record launch-pad pressure as the barometer's zero-altitude
         * reference.  BMP388_ReadAltitude() will then return AGL altitude.
         */
        sensor_status = BMP388_CalibrateAltitudeReference(
            &rocket_baro,
            BEHAVIOR_BARO_REFERENCE_SAMPLES,
            BEHAVIOR_BARO_REFERENCE_DELAY_MS);
    }

    /* Calibration is intentionally blocking, so reset all timing from the
     * actual completion time instead of the timestamp passed at entry.
     */
    init_complete_ms = HAL_GetTick();

    Behavior_ResetFusion();
    Behavior_ResetEstimatorController(init_complete_ms);

    telemetry.initialized = true;
    telemetry.status = (sensor_status == HAL_OK)
        ? BEHAVIOR_STATUS_OK
        : BEHAVIOR_STATUS_SENSOR_INIT_FAILED;

    Behavior_CopyConfigToTelemetry();
    Behavior_ResetSyntheticTest(init_complete_ms, true);
    Behavior_EnterMode(BEHAVIOR_MODE_STANDARD, 0U, init_complete_ms);

    /* Behavior_EnterMode() sets status to OK, so restore an initialization
     * failure after all state initialization is complete.
     */
    telemetry.status = (sensor_status == HAL_OK)
        ? BEHAVIOR_STATUS_OK
        : BEHAVIOR_STATUS_SENSOR_INIT_FAILED;

    return sensor_status;
}

// input config into starting values for everything it applies to
BehaviorStatus_t Behavior_ApplyConfig(const BehaviorConfig_t *config,
                                      uint32_t now_ms)
{
    if (!Behavior_ValidateConfig(config))
    {
        telemetry.status = BEHAVIOR_STATUS_INVALID_CONFIG;
        return telemetry.status;
    }

    behavior_config = *config;
    Behavior_CopyConfigToTelemetry();
    Behavior_ResetFusion();
    Behavior_ResetEstimatorController(now_ms);
    Behavior_ResetSyntheticTest(now_ms, false);

    telemetry.status = BEHAVIOR_STATUS_OK;
    return telemetry.status;
}

BehaviorStatus_t Behavior_SetTargetApogee(float target_apogee_m,
                                          uint32_t now_ms)
{
    if (!Behavior_Finite(target_apogee_m) ||
        (target_apogee_m <= behavior_config.initial_altitude_agl_m))
    {
        telemetry.status = BEHAVIOR_STATUS_INVALID_CONFIG;
        return telemetry.status;
    }

    behavior_config.target_apogee_m = target_apogee_m;
    Controller_Init(&controller, target_apogee_m);
    Behavior_CopyConfigToTelemetry();
    (void)now_ms;

    telemetry.status = BEHAVIOR_STATUS_OK;
    return telemetry.status;
}

BehaviorStatus_t Behavior_SetInitialAltitudeAGL(float initial_altitude_agl_m,
                                                uint32_t now_ms)
{
    if (!Behavior_Finite(initial_altitude_agl_m) ||
        (initial_altitude_agl_m >= behavior_config.target_apogee_m))
    {
        telemetry.status = BEHAVIOR_STATUS_INVALID_CONFIG;
        return telemetry.status;
    }

    behavior_config.initial_altitude_agl_m = initial_altitude_agl_m;
    Behavior_CopyConfigToTelemetry();
    Behavior_ResetEstimatorController(now_ms);

    telemetry.status = BEHAVIOR_STATUS_OK;
    return telemetry.status;
}

BehaviorStatus_t Behavior_SetDeploymentBounds(float deployment_min,
                                              float deployment_max)
{
    if (!Behavior_Finite(deployment_min) ||
        !Behavior_Finite(deployment_max) ||
        (deployment_min < 0.0f) ||
        (deployment_max > 1.0f) ||
        (deployment_min > deployment_max))
    {
        telemetry.status = BEHAVIOR_STATUS_INVALID_CONFIG;
        return telemetry.status;
    }

    behavior_config.deployment_min = deployment_min;
    behavior_config.deployment_max = deployment_max;
    Behavior_CopyConfigToTelemetry();

    telemetry.status = BEHAVIOR_STATUS_OK;
    return telemetry.status;
}

BehaviorStatus_t Behavior_SetControllerEnabled(bool enabled)
{
    behavior_config.controller_enabled = enabled;
    Behavior_CopyConfigToTelemetry();

    if (!enabled)
    {
        telemetry.controller_raw_deployment = 0.0f;
        telemetry.controller_requested_deployment = 0.0f;
        telemetry.controller_requested_percent = 0U;
        telemetry.controller_active = false;
    }

    telemetry.status = BEHAVIOR_STATUS_OK;
    return telemetry.status;
}

BehaviorStatus_t Behavior_SubmitBarometerAltitudeAGL(float altitude_agl_m,
                                                     uint32_t now_ms)
{
    if (!Behavior_Finite(altitude_agl_m))
    {
        telemetry.status = BEHAVIOR_STATUS_BAD_ARGUMENT;
        return telemetry.status;
    }

    submitted_baro_altitude_agl_m = altitude_agl_m;
    submitted_baro_timestamp_ms = now_ms;
    submitted_baro_sequence++;

    telemetry.barometer_altitude_agl_m = altitude_agl_m;
    telemetry.barometer_altitude_timestamp_ms = now_ms;
    telemetry.barometer_altitude_valid = true;

    return BEHAVIOR_STATUS_OK;
}

BehaviorStatus_t Behavior_SubmitBarometerAltitudeMSL(float altitude_msl_m,
                                                     uint32_t now_ms)
{
    if (!Behavior_Finite(altitude_msl_m))
    {
        telemetry.status = BEHAVIOR_STATUS_BAD_ARGUMENT;
        return telemetry.status;
    }

    return Behavior_SubmitBarometerAltitudeAGL(
        altitude_msl_m - behavior_config.launch_site_altitude_msl_m,
        now_ms);
}

// step the current behavior. no logic is stored here, different modes will have different behaviors
void Behavior_Update(uint32_t now_ms)
{
    telemetry.mode_changed = false;
    telemetry.barometer_correction_used = false;

    if (!telemetry.initialized)
    {
        telemetry.status = BEHAVIOR_STATUS_BAD_ARGUMENT;
        return;
    }

    if (telemetry.auto_return_enabled &&
        ((uint32_t)(now_ms - telemetry.mode_entered_ms) >=
         telemetry.auto_return_after_ms))
    {
        Behavior_ReturnToStandard(now_ms);
    }

    switch (telemetry.mode)
    {
        case BEHAVIOR_MODE_STANDARD:
            Behavior_UpdateStandard(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_RAW_SENSORS:
            Behavior_UpdateRawSensorTest(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_SYNTHETIC_EKF:
            Behavior_UpdateSyntheticEkfTest(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_FUSION:
            Behavior_UpdateFusionTest(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_REAL_IMU_EKF:
            Behavior_UpdateRealImuEkfTest(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_FULL_PIPELINE:
            Behavior_UpdateFullPipelineTest(now_ms);
            break;

        default:
            telemetry.status = BEHAVIOR_STATUS_UNSUPPORTED_MODE;
            Behavior_ReturnToStandard(now_ms);
            break;
    }

    /*
     * Physical airbrake and motor output remains disabled.
     * controller_requested_percent is available for inspection/telemetry.
     */
    //Airbrake_Update(now_ms);
    //Airbrake_SetTargetPercent(telemetry.controller_requested_percent);
    // TMC5240_EnableDriver();
}

// attempt to set the mode
BehaviorStatus_t Behavior_RequestMode(BehaviorMode_t mode,
                                      uint32_t duration_ms,
                                      uint32_t now_ms)
{
    if (!Behavior_IsSupportedMode(mode))
    {
        telemetry.status = BEHAVIOR_STATUS_UNSUPPORTED_MODE;
        return telemetry.status;
    }

    Behavior_EnterMode(mode, duration_ms, now_ms);
    return BEHAVIOR_STATUS_OK;
}

void Behavior_ReturnToStandard(uint32_t now_ms)
{
    Behavior_EnterMode(BEHAVIOR_MODE_STANDARD, 0U, now_ms);
}

BehaviorMode_t Behavior_GetMode(void)
{
    return telemetry.mode;
}

const BehaviorConfig_t *Behavior_GetConfig(void)
{
    return &behavior_config;
}

const volatile BehaviorTelemetry_t *Behavior_GetTelemetry(void)
{
    return &telemetry;
}

static void Behavior_EnterMode(BehaviorMode_t mode,
                               uint32_t duration_ms,
                               uint32_t now_ms)
{
    telemetry.previous_mode = telemetry.mode;
    telemetry.mode = mode;
    telemetry.mode_changed = true;
    telemetry.mode_entered_ms = now_ms;
    telemetry.auto_return_after_ms = duration_ms;
    telemetry.auto_return_enabled = (duration_ms > 0U);
    telemetry.status = BEHAVIOR_STATUS_OK;

    switch (mode)
    {
        case BEHAVIOR_MODE_STANDARD:
            Behavior_ResetFusion();
            Behavior_ResetEstimatorController(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_RAW_SENSORS:
            last_sensor_attempt_ms = now_ms;
            last_sensor_update_ms = now_ms;
            break;

        case BEHAVIOR_MODE_TEST_SYNTHETIC_EKF:
            Behavior_ResetEstimatorController(now_ms);
            Behavior_ResetSyntheticTest(now_ms, false);
            break;

        case BEHAVIOR_MODE_TEST_FUSION:
            Behavior_ResetFusion();
            last_sensor_attempt_ms = now_ms;
            last_sensor_update_ms = now_ms;
            break;

        case BEHAVIOR_MODE_TEST_REAL_IMU_EKF:
            Behavior_ResetFusion();
            Behavior_ResetEstimatorController(now_ms);
            break;

        case BEHAVIOR_MODE_TEST_FULL_PIPELINE:
            Behavior_ResetFusion();
            Behavior_ResetEstimatorController(now_ms);
            Behavior_ResetSyntheticTest(now_ms, false);
            break;

        default:
            break;
    }
}

// verify if mode actually exists
static bool Behavior_IsSupportedMode(BehaviorMode_t mode)
{
    return (mode >= BEHAVIOR_MODE_STANDARD) &&
           (mode <= BEHAVIOR_MODE_TEST_FULL_PIPELINE);
}

// verify if config is possible
static bool Behavior_ValidateConfig(const BehaviorConfig_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    return Behavior_Finite(config->target_apogee_m) &&
           Behavior_Finite(config->initial_altitude_agl_m) &&
           Behavior_Finite(config->launch_site_altitude_msl_m) &&
           Behavior_Finite(config->deployment_min) &&
           Behavior_Finite(config->deployment_max) &&
           Behavior_Finite(config->estimator_rate_hz) &&
           Behavior_Finite(config->altitude_tolerance_m) &&
           (config->target_apogee_m > config->initial_altitude_agl_m) &&
           (config->deployment_min >= 0.0f) &&
           (config->deployment_max <= 1.0f) &&
           (config->deployment_min <= config->deployment_max) &&
           (config->estimator_rate_hz >= 20.0f) &&
           (config->estimator_rate_hz <= 1000.0f) &&
           (config->altitude_tolerance_m > 0.0f) &&
           (config->synthetic_pad_warmup_s >= 3.0f) &&
           (config->synthetic_burn_time_s > 0.0f) &&
           (config->synthetic_base_drag_k >= 0.0f) &&
           (config->synthetic_airbrake_drag_k >= 0.0f);
}

// push config to telemetry struct
static void Behavior_CopyConfigToTelemetry(void)
{
    telemetry.target_apogee_m = behavior_config.target_apogee_m;
    telemetry.initial_altitude_agl_m = behavior_config.initial_altitude_agl_m;
    telemetry.launch_site_altitude_msl_m = behavior_config.launch_site_altitude_msl_m;
    telemetry.deployment_min = behavior_config.deployment_min;
    telemetry.deployment_max = behavior_config.deployment_max;
    telemetry.altitude_tolerance_m = behavior_config.altitude_tolerance_m;
    telemetry.controller_enabled = behavior_config.controller_enabled;
}

static void Behavior_ResetFusion(void)
{
    FusionBiasSettings bias_settings;
    FusionAhrsSettings ahrs_settings;

    bias_settings.sampleRate = behavior_config.estimator_rate_hz;
    bias_settings.stationaryThreshold = 3.0f;
    bias_settings.stationaryPeriod = 3.0f;

    ahrs_settings.sampleRate = behavior_config.estimator_rate_hz;
    ahrs_settings.convention = behavior_config.fusion_convention;
    ahrs_settings.gain = behavior_config.fusion_gain;
    ahrs_settings.gyroscopeRange = behavior_config.gyroscope_range_dps;
    ahrs_settings.accelerationRejection = behavior_config.acceleration_rejection_deg;
    ahrs_settings.magneticRejection = behavior_config.magnetic_rejection_deg;
    ahrs_settings.recoveryTriggerPeriod =
        (unsigned int)(behavior_config.fusion_recovery_period_s *
                       behavior_config.estimator_rate_hz);

    FusionBiasInitialise(&fusion_bias);
    FusionBiasSetSettings(&fusion_bias, &bias_settings);

    FusionAhrsInitialise(&fusion_ahrs);
    FusionAhrsSetSettings(&fusion_ahrs, &ahrs_settings);

    latest_magnetometer = FUSION_VECTOR_ZERO;
    latest_magnetometer_valid = false;
}

static void Behavior_ResetEstimatorController(uint32_t now_ms)
{
    AltitudeEKF_Init(&altitude_ekf,
                     behavior_config.initial_altitude_agl_m,
                     BEHAVIOR_EKF_VAR_BARO,
                     BEHAVIOR_EKF_VAR_IMU_ACCEL,
                     BEHAVIOR_EKF_VAR_PROCESS_JERK,
                     BEHAVIOR_EKF_VAR_PROCESS_BIAS);

    Controller_Init(&controller, behavior_config.target_apogee_m);

    last_sensor_attempt_ms = now_ms;
    last_sensor_update_ms = now_ms;
    consumed_baro_sequence = submitted_baro_sequence;

    telemetry.ekf_altitude_m = behavior_config.initial_altitude_agl_m;
    telemetry.ekf_velocity_m_s = 0.0f;
    telemetry.ekf_acceleration_m_s2 = 0.0f;
    telemetry.ekf_bias_m_s2 = 0.0f;

    telemetry.controller_raw_deployment = 0.0f;
    telemetry.controller_requested_deployment = 0.0f;
    telemetry.controller_requested_percent = 0U;
    telemetry.controller_active = false;
}

static void Behavior_ResetSyntheticTest(uint32_t now_ms,
                                        bool reset_fusion)
{
    if (reset_fusion)
    {
        Behavior_ResetFusion();
    }

    synthetic_time_s = 0.0f;
    synthetic_previous_velocity_m_s = 0.0f;
    synthetic_baro_counter = 0U;
    synthetic_ekf_elapsed_s = 0.0f;
    synthetic_accel_sum_m_s2 = 0.0f;
    synthetic_accel_count = 0U;
    synthetic_truth_quaternion = FUSION_QUATERNION_IDENTITY;
    last_synthetic_update_ms = now_ms;

    telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_PAD;
    telemetry.test_time_s = 0.0f;
    telemetry.test_flight_time_s = 0.0f;
    telemetry.test_true_altitude_m = behavior_config.initial_altitude_agl_m;
    telemetry.test_true_velocity_m_s = 0.0f;
    telemetry.test_true_acceleration_m_s2 = 0.0f;
    telemetry.test_roll_deg = 0.0f;
    telemetry.test_pitch_deg = 0.0f;
    telemetry.test_yaw_deg = 0.0f;

    telemetry.fusion_roll_error_deg = 0.0f;
    telemetry.fusion_pitch_error_deg = 0.0f;
    telemetry.fusion_yaw_error_deg = 0.0f;
    telemetry.max_fusion_roll_error_deg = 0.0f;
    telemetry.max_fusion_pitch_error_deg = 0.0f;
    telemetry.max_fusion_yaw_error_deg = 0.0f;

    telemetry.altitude_error_m = 0.0f;
    telemetry.velocity_error_m_s = 0.0f;
    telemetry.max_altitude_error_m = 0.0f;
    telemetry.max_velocity_error_m_s = 0.0f;

    telemetry.true_apogee_m = behavior_config.initial_altitude_agl_m;
    telemetry.estimated_apogee_m = behavior_config.initial_altitude_agl_m;
    telemetry.true_apogee_error_m = 0.0f;
    telemetry.apogee_reached = false;

    telemetry.fusion_data_valid = false;
    telemetry.fusion_within_tolerance = false;
    telemetry.ekf_data_valid = false;
    telemetry.ekf_within_altitude_tolerance = false;
    telemetry.controller_output_valid = false;
    telemetry.controller_apogee_within_tolerance = false;
    telemetry.full_pipeline_complete = false;
    telemetry.full_pipeline_pass = false;

    telemetry.synthetic_samples = 0U;
    telemetry.full_pipeline_samples = 0U;
}

static void Behavior_UpdateStandard(uint32_t now_ms)
{
    Behavior_UpdateRealPipeline(now_ms, true, true);
}
// step sensor measurement
static void Behavior_UpdateRawSensorTest(uint32_t now_ms)
{
    float dt_s;
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionVector magnetometer;

    if (!Behavior_ReadAndConvertSensors(now_ms, &dt_s))
    {
        return;
    }

    Behavior_ConvertRawSensors(&gyroscope, &accelerometer, &magnetometer);
    (void)dt_s;

    telemetry.controller_active = false;
    telemetry.controller_requested_deployment = 0.0f;
    telemetry.controller_requested_percent = 0U;
}
// step synthetic data for ekf
static void Behavior_UpdateSyntheticEkfTest(uint32_t now_ms)
{
    const uint32_t period_ms = Behavior_GetUpdatePeriodMilliseconds();
    const float dt_s = Behavior_GetUpdatePeriodSeconds();
    float measured_acceleration;
    float synchronized_acceleration;

    if ((uint32_t)(now_ms - last_synthetic_update_ms) < period_ms)
    {
        return;
    }

    last_synthetic_update_ms += period_ms;

    if (telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_LANDED)
    {
        return;
    }

    Behavior_AdvanceSyntheticDynamics(dt_s);

    measured_acceleration = telemetry.test_true_acceleration_m_s2 +
                            BEHAVIOR_SYNTH_ACCEL_BIAS_M_S2 +
                            Behavior_Noise(BEHAVIOR_SYNTH_ACCEL_NOISE_M_S2);

    telemetry.vertical_acceleration_m_s2 = measured_acceleration;
    telemetry.synthetic_samples++;
    telemetry.fusion_data_valid = false;
    telemetry.fusion_within_tolerance = false;

    /* The synthetic IMU is faster than the synthetic barometer.  Accumulate
     * IMU measurements, but do not update the EKF until a new barometer sample
     * is available so the test follows the same synchronization rule as the
     * real pipeline.
     */
    synthetic_ekf_elapsed_s += dt_s;
    synthetic_accel_sum_m_s2 += measured_acceleration;
    synthetic_accel_count++;
    synthetic_baro_counter++;

    if (synthetic_baro_counter < BEHAVIOR_SYNTHETIC_BARO_DIVIDER)
    {
        return;
    }

    synthetic_baro_counter = 0U;
    synchronized_acceleration = synthetic_accel_sum_m_s2 /
                                (float)synthetic_accel_count;

    AltitudeEKF_UpdateImu(&altitude_ekf,
                          synchronized_acceleration,
                          synthetic_ekf_elapsed_s);
    AltitudeEKF_UpdateBaro(
        &altitude_ekf,
        telemetry.test_true_altitude_m +
        Behavior_Noise(BEHAVIOR_SYNTH_BARO_NOISE_M));
    telemetry.barometer_correction_used = true;

    Behavior_CopyEkfTelemetry();
    Behavior_UpdateController(now_ms,
        synthetic_ekf_elapsed_s,
        telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_COAST);

    synthetic_ekf_elapsed_s = 0.0f;
    synthetic_accel_sum_m_s2 = 0.0f;
    synthetic_accel_count = 0U;

    Behavior_UpdateTestMetrics();
}
// synthetic fusion test
static void Behavior_UpdateFusionTest(uint32_t now_ms)
{
    float dt_s;
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionVector magnetometer;

    if (!Behavior_ReadAndConvertSensors(now_ms, &dt_s))
    {
        return;
    }

    Behavior_ConvertRawSensors(&gyroscope, &accelerometer, &magnetometer);
    Behavior_UpdateFusion(gyroscope,
                          accelerometer,
                          magnetometer,
                          latest_magnetometer_valid,
                          dt_s);

    telemetry.controller_active = false;
    telemetry.controller_requested_deployment = 0.0f;
    telemetry.controller_requested_percent = 0U;
}
// update IMU ekf
static void Behavior_UpdateRealImuEkfTest(uint32_t now_ms)
{
    Behavior_UpdateRealPipeline(now_ms, true, true);
}
// pipeline test step
static void Behavior_UpdateFullPipelineTest(uint32_t now_ms)
{
    const uint32_t period_ms = Behavior_GetUpdatePeriodMilliseconds();
    const float dt_s = Behavior_GetUpdatePeriodSeconds();
    FusionVector body_rate_truth;
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionVector magnetometer;
    float synchronized_acceleration;

    if ((uint32_t)(now_ms - last_synthetic_update_ms) < period_ms)
    {
        return;
    }

    last_synthetic_update_ms += period_ms;

    if (telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_LANDED)
    {
        return;
    }

    Behavior_AdvanceSyntheticDynamics(dt_s);

    body_rate_truth = Behavior_GetSyntheticBodyRate();
    Behavior_UpdateTruthQuaternion(body_rate_truth, dt_s);
    Behavior_CreateSyntheticSensors(body_rate_truth,
                                    &gyroscope,
                                    &accelerometer,
                                    &magnetometer);
    Behavior_UpdateSyntheticTruthTelemetry(gyroscope,
                                           accelerometer,
                                           magnetometer);

    /* Fusion may run at the faster IMU rate.  Only the EKF is locked to the
     * slowest sensor, matching the real-sensor pipeline.
     */
    Behavior_UpdateFusion(gyroscope,
                          accelerometer,
                          magnetometer,
                          true,
                          dt_s);

    telemetry.full_pipeline_samples++;
    synthetic_ekf_elapsed_s += dt_s;
    synthetic_accel_sum_m_s2 += telemetry.vertical_acceleration_m_s2;
    synthetic_accel_count++;
    synthetic_baro_counter++;

    if (synthetic_baro_counter < BEHAVIOR_SYNTHETIC_BARO_DIVIDER)
    {
        return;
    }

    synthetic_baro_counter = 0U;
    synchronized_acceleration = synthetic_accel_sum_m_s2 /
                                (float)synthetic_accel_count;

    AltitudeEKF_UpdateImu(&altitude_ekf,
                          synchronized_acceleration,
                          synthetic_ekf_elapsed_s);
    AltitudeEKF_UpdateBaro(
        &altitude_ekf,
        telemetry.test_true_altitude_m +
        Behavior_Noise(BEHAVIOR_SYNTH_BARO_NOISE_M));
    telemetry.barometer_correction_used = true;

    Behavior_CopyEkfTelemetry();
    Behavior_UpdateController(now_ms,
        synthetic_ekf_elapsed_s,
        telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_COAST);

    synthetic_ekf_elapsed_s = 0.0f;
    synthetic_accel_sum_m_s2 = 0.0f;
    synthetic_accel_count = 0U;

    Behavior_UpdateTestMetrics();
}
// collect raw data
static bool Behavior_ReadAndConvertSensors(uint32_t now_ms,
                                           float *dt_s)
{
    const uint32_t poll_period_ms = Behavior_GetUpdatePeriodMilliseconds();
    uint32_t elapsed_ms;
    uint8_t imu_status_reg = 0U;
    uint8_t mag_status_reg = 0U;
    uint8_t baro_status_reg = 0U;
    float imu_sample[LSM6DSV32X_DATA_COUNT];
    float mag_sample[LIS2MDL_DATA_COUNT];
    float barometer_altitude_agl_m;
    HAL_StatusTypeDef status;

    if (dt_s == NULL)
    {
        telemetry.status = BEHAVIOR_STATUS_BAD_ARGUMENT;
        return false;
    }

    /* Poll at the requested estimator rate, but do not advance the EKF clock
     * until every sensor reports a new sample.  The slowest sensor therefore
     * controls the actual EKF update rate.
     */
    if ((uint32_t)(now_ms - last_sensor_attempt_ms) < poll_period_ms)
    {
        return false;
    }
    last_sensor_attempt_ms = now_ms;

    status = LSM6DSV32X_ReadStatus(&rocket_imu, &imu_status_reg);
    if (status != HAL_OK)
    {
        telemetry.imu_status = status;
        telemetry.failed_reads++;
        telemetry.status = BEHAVIOR_STATUS_SENSOR_READ_FAILED;
        return false;
    }

    status = LIS2MDL_ReadStatus(&rocket_mag, &mag_status_reg);
    if (status != HAL_OK)
    {
        telemetry.mag_status = status;
        telemetry.failed_reads++;
        telemetry.status = BEHAVIOR_STATUS_SENSOR_READ_FAILED;
        return false;
    }

    status = BMP388_ReadStatus(&rocket_baro, &baro_status_reg);
    if (status != HAL_OK)
    {
        telemetry.baro_status = status;
        telemetry.failed_reads++;
        telemetry.status = BEHAVIOR_STATUS_SENSOR_READ_FAILED;
        return false;
    }

    telemetry.imu_status =
        ((imu_status_reg & BEHAVIOR_IMU_READY_MASK) ==
         BEHAVIOR_IMU_READY_MASK) ? HAL_OK : HAL_BUSY;
    telemetry.mag_status =
        ((mag_status_reg & BEHAVIOR_MAG_READY_MASK) ==
         BEHAVIOR_MAG_READY_MASK) ? HAL_OK : HAL_BUSY;
    telemetry.baro_status =
        ((baro_status_reg & BEHAVIOR_BARO_READY_MASK) ==
         BEHAVIOR_BARO_READY_MASK) ? HAL_OK : HAL_BUSY;

    /* A HAL_BUSY value here means the bus transaction succeeded, but that
     * sensor has not produced a new measurement yet.  Do not consume any of
     * the other samples and do not touch Fusion, the EKF, or the controller.
     */
    if ((telemetry.imu_status != HAL_OK) ||
        (telemetry.mag_status != HAL_OK) ||
        (telemetry.baro_status != HAL_OK))
    {
        telemetry.status = BEHAVIOR_STATUS_OK;
        return false;
    }

    telemetry.imu_status = LSM6DSV32X_ReadData(
        &rocket_imu,
        imu_sample);
    if (telemetry.imu_status == HAL_OK)
    {
        telemetry.imu_samples++;
    }

    telemetry.mag_status = LIS2MDL_ReadData(
        &rocket_mag,
        mag_sample);
    if (telemetry.mag_status == HAL_OK)
    {
        telemetry.mag_samples++;
    }

    /* Read compensated pressure internally and convert it to altitude using
     * the launch-pad reference captured during Behavior_Init().
     */
    telemetry.baro_status = BMP388_ReadAltitude(
        &rocket_baro,
        &barometer_altitude_agl_m);
    if (telemetry.baro_status == HAL_OK)
    {
        barometer_altitude_agl_m += behavior_config.initial_altitude_agl_m;
        telemetry.baro_samples++;
    }

    /* The EKF update is atomic with respect to the sensor set.  A partial read
     * is never accepted, even if one or two sensors succeeded.
     */
    if ((telemetry.imu_status != HAL_OK) ||
        (telemetry.mag_status != HAL_OK) ||
        (telemetry.baro_status != HAL_OK) ||
        !Behavior_Finite(barometer_altitude_agl_m))
    {
        telemetry.failed_reads++;
        telemetry.status = BEHAVIOR_STATUS_SENSOR_READ_FAILED;
        return false;
    }

    /* Commit the sensor set only after all three reads succeeded. */
    memcpy((void *)telemetry.raw.imu, imu_sample, sizeof(imu_sample));
    memcpy((void *)telemetry.raw.mag, mag_sample, sizeof(mag_sample));

    submitted_baro_altitude_agl_m = barometer_altitude_agl_m;
    submitted_baro_timestamp_ms = now_ms;
    submitted_baro_sequence++;

    telemetry.barometer_altitude_agl_m = barometer_altitude_agl_m;
    telemetry.barometer_altitude_timestamp_ms = now_ms;
    telemetry.barometer_altitude_valid = true;

    elapsed_ms = (uint32_t)(now_ms - last_sensor_update_ms);
    last_sensor_update_ms = now_ms;

    *dt_s = (float)elapsed_ms * 0.001f;
    *dt_s = Behavior_ClampFloat(*dt_s,
                                BEHAVIOR_MIN_DT_S,
                                BEHAVIOR_MAX_DT_S);

    telemetry.status = BEHAVIOR_STATUS_OK;
    return true;
}
// raw data to usable data
static void Behavior_ConvertRawSensors(FusionVector *gyroscope,
                                       FusionVector *accelerometer,
                                       FusionVector *magnetometer)
{
    FusionVector gyro_unmapped;
    FusionVector accel_unmapped;
    FusionVector mag_uncalibrated;

    if ((gyroscope == NULL) ||
        (accelerometer == NULL) ||
        (magnetometer == NULL))
    {
        return;
    }

    gyro_unmapped.axis.x =
        (float)telemetry.raw.imu[LSM6DSV32X_GYRO_X] *
        behavior_config.gyro_dps_per_lsb -
        behavior_config.gyro_offset_dps.axis.x;
    gyro_unmapped.axis.y =
        (float)telemetry.raw.imu[LSM6DSV32X_GYRO_Y] *
        behavior_config.gyro_dps_per_lsb -
        behavior_config.gyro_offset_dps.axis.y;
    gyro_unmapped.axis.z =
        (float)telemetry.raw.imu[LSM6DSV32X_GYRO_Z] *
        behavior_config.gyro_dps_per_lsb -
        behavior_config.gyro_offset_dps.axis.z;

    accel_unmapped.axis.x =
        (float)telemetry.raw.imu[LSM6DSV32X_ACCEL_X] *
        behavior_config.accel_g_per_lsb -
        behavior_config.accel_offset_g.axis.x;
    accel_unmapped.axis.y =
        (float)telemetry.raw.imu[LSM6DSV32X_ACCEL_Y] *
        behavior_config.accel_g_per_lsb -
        behavior_config.accel_offset_g.axis.y;
    accel_unmapped.axis.z =
        (float)telemetry.raw.imu[LSM6DSV32X_ACCEL_Z] *
        behavior_config.accel_g_per_lsb -
        behavior_config.accel_offset_g.axis.z;

    *gyroscope = FusionRemap(gyro_unmapped,
                             behavior_config.imu_alignment);
    *accelerometer = FusionRemap(accel_unmapped,
                                 behavior_config.imu_alignment);

    if (telemetry.mag_status == HAL_OK)
    {
        mag_uncalibrated.axis.x = (float)telemetry.raw.mag[LIS2MDL_MAG_X];
        mag_uncalibrated.axis.y = (float)telemetry.raw.mag[LIS2MDL_MAG_Y];
        mag_uncalibrated.axis.z = (float)telemetry.raw.mag[LIS2MDL_MAG_Z];

        latest_magnetometer = FusionModelMagnetic(
            mag_uncalibrated,
            behavior_config.mag_soft_iron_matrix,
            behavior_config.mag_hard_iron_offset);
        latest_magnetometer = FusionRemap(latest_magnetometer,
                                          behavior_config.mag_alignment);
        latest_magnetometer_valid = true;
    }

    *magnetometer = latest_magnetometer;

    telemetry.gyro_dps[0] = gyroscope->axis.x;
    telemetry.gyro_dps[1] = gyroscope->axis.y;
    telemetry.gyro_dps[2] = gyroscope->axis.z;

    telemetry.accel_g[0] = accelerometer->axis.x;
    telemetry.accel_g[1] = accelerometer->axis.y;
    telemetry.accel_g[2] = accelerometer->axis.z;

    telemetry.mag[0] = magnetometer->axis.x;
    telemetry.mag[1] = magnetometer->axis.y;
    telemetry.mag[2] = magnetometer->axis.z;
}
// step fusion process
static void Behavior_UpdateFusion(const FusionVector gyroscope,
                                  const FusionVector accelerometer,
                                  const FusionVector magnetometer,
                                  bool magnetometer_valid,
                                  float dt_s)
{
    FusionVector corrected_gyroscope;
    FusionVector earth_acceleration;

    corrected_gyroscope = FusionBiasUpdate(&fusion_bias, gyroscope);
    FusionAhrsSetSamplePeriod(&fusion_ahrs, dt_s);

    if (magnetometer_valid)
    {
        FusionAhrsUpdate(&fusion_ahrs,
                         corrected_gyroscope,
                         accelerometer,
                         magnetometer);
    }
    else
    {
        FusionAhrsUpdateNoMagnetometer(&fusion_ahrs,
                                       corrected_gyroscope,
                                       accelerometer);
    }

    earth_acceleration = FusionAhrsGetEarthAcceleration(&fusion_ahrs);
    Behavior_CopyFusionTelemetry(earth_acceleration);
}

// step rocket process
static void Behavior_UpdateRealPipeline(uint32_t now_ms,
                                        bool update_ekf,
                                        bool update_controller)
{
    float dt_s;
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionVector magnetometer;

    if (!Behavior_ReadAndConvertSensors(now_ms, &dt_s))
    {
        return;
    }

    Behavior_ConvertRawSensors(&gyroscope, &accelerometer, &magnetometer);
    Behavior_UpdateFusion(gyroscope,
                          accelerometer,
                          magnetometer,
                          latest_magnetometer_valid,
                          dt_s);

    if (update_ekf)
    {
        /* Behavior_ReadAndConvertSensors() returns true only when IMU,
         * magnetometer, and barometer are all fresh.  Keep the IMU prediction
         * and barometer correction in the same synchronized EKF transaction.
         */
        AltitudeEKF_UpdateImu(&altitude_ekf,
                              telemetry.vertical_acceleration_m_s2,
                              dt_s);
        Behavior_ConsumeBarometerCorrection(now_ms);
        Behavior_CopyEkfTelemetry();
    }

    if (update_controller && update_ekf)
    {
        Behavior_UpdateController(now_ms, dt_s, true);
    }
    else
    {
        telemetry.controller_active = false;
    }
}
// push barometer to ekf
static void Behavior_ConsumeBarometerCorrection(uint32_t now_ms)
{
    if ((submitted_baro_sequence == consumed_baro_sequence) ||
        ((uint32_t)(now_ms - submitted_baro_timestamp_ms) >
         BEHAVIOR_BARO_STALE_MS))
    {
        return;
    }

    consumed_baro_sequence = submitted_baro_sequence;
    AltitudeEKF_UpdateBaro(&altitude_ekf,
                          submitted_baro_altitude_agl_m);

    telemetry.barometer_altitude_agl_m = submitted_baro_altitude_agl_m;
    telemetry.barometer_altitude_timestamp_ms = submitted_baro_timestamp_ms;
    telemetry.barometer_altitude_valid = true;
    telemetry.barometer_correction_used = true;
}
// step controller
static void Behavior_UpdateController(uint32_t now_ms, float dt_s,
                                      bool allow_deployment)
{
    ControllerData controller_data;
    float requested;

    telemetry.predicted_apogee_m = telemetry.ekf_altitude_m;
    if (telemetry.ekf_velocity_m_s > 0.0f)
    {
        telemetry.predicted_apogee_m +=
            (telemetry.ekf_velocity_m_s * telemetry.ekf_velocity_m_s) /
            (2.0f * BEHAVIOR_GRAVITY_M_S2);
    }

    if (!behavior_config.controller_enabled || !allow_deployment)
    {
        Controller_Close(&controller, dt_s);
        telemetry.controller_raw_deployment =
            Controller_GetDeployment(&controller);
        telemetry.controller_requested_deployment = 0.0f;
        telemetry.controller_requested_percent = 0U;
        telemetry.controller_active = false;
        telemetry.controller_output_valid = true;
        return;
    }

    controller_data.altitude_agl = telemetry.ekf_altitude_m;
    controller_data.vertical_velocity = telemetry.ekf_velocity_m_s;
    controller_data.vertical_acceleration = telemetry.ekf_acceleration_m_s2;

    telemetry.controller_raw_deployment = Controller_NewDeployment(
        &controller,
        controller_data,
        dt_s);

    requested = Behavior_ClampFloat(
        telemetry.controller_raw_deployment,
        behavior_config.deployment_min,
        behavior_config.deployment_max);

    telemetry.controller_requested_deployment = requested;
    telemetry.controller_requested_percent =
        (uint8_t)(requested * 100.0f + 0.5f);
    telemetry.controller_active = true;
    telemetry.controller_output_valid =
        Behavior_Finite(requested) &&
        (requested >= behavior_config.deployment_min) &&
        (requested <= behavior_config.deployment_max);

    /* Physical command intentionally disabled. */
    Airbrake_Update(now_ms);
    Airbrake_SetTargetPercent(telemetry.controller_requested_percent);
}
// push ekf to telemetry struct
static void Behavior_CopyEkfTelemetry(void)
{
    telemetry.ekf_altitude_m =
        AltitudeEKF_GetAltitudeAGL(&altitude_ekf);
    telemetry.ekf_velocity_m_s =
        AltitudeEKF_GetVerticalVelocity(&altitude_ekf);
    telemetry.ekf_acceleration_m_s2 =
        AltitudeEKF_GetVerticalAcceleration(&altitude_ekf);
    telemetry.ekf_bias_m_s2 =
        AltitudeEKF_GetAccelBias(&altitude_ekf);

    telemetry.ekf_data_valid =
        Behavior_Finite(telemetry.ekf_altitude_m) &&
        Behavior_Finite(telemetry.ekf_velocity_m_s) &&
        Behavior_Finite(telemetry.ekf_acceleration_m_s2) &&
        Behavior_Finite(telemetry.ekf_bias_m_s2);
}
// push fusion to telemetry struct
static void Behavior_CopyFusionTelemetry(const FusionVector earth_acceleration)
{
    const FusionEuler euler = FusionQuaternionToEuler(
        FusionAhrsGetQuaternion(&fusion_ahrs));
    const FusionAhrsInternalStates states =
        FusionAhrsGetInternalStates(&fusion_ahrs);
    const FusionAhrsFlags flags = FusionAhrsGetFlags(&fusion_ahrs);

    telemetry.fusion_roll_deg = euler.angle.roll;
    telemetry.fusion_pitch_deg = euler.angle.pitch;
    telemetry.fusion_yaw_deg = euler.angle.yaw;

    telemetry.fusion_earth_accel_g[0] = earth_acceleration.axis.x;
    telemetry.fusion_earth_accel_g[1] = earth_acceleration.axis.y;
    telemetry.fusion_earth_accel_g[2] = earth_acceleration.axis.z;

    telemetry.vertical_acceleration_m_s2 =
        Behavior_EarthVerticalToPositiveUp(earth_acceleration);

    telemetry.fusion_acceleration_error_deg = states.accelerationError;
    telemetry.fusion_magnetic_error_deg = states.magneticError;
    telemetry.fusion_accelerometer_ignored = states.accelerometerIgnored;
    telemetry.fusion_magnetometer_ignored = states.magnetometerIgnored;
    telemetry.fusion_startup = flags.startup;

    telemetry.fusion_data_valid =
        Behavior_Finite(telemetry.fusion_roll_deg) &&
        Behavior_Finite(telemetry.fusion_pitch_deg) &&
        Behavior_Finite(telemetry.fusion_yaw_deg) &&
        Behavior_Finite(telemetry.vertical_acceleration_m_s2);
}
// detect flight steps
static void Behavior_AdvanceSyntheticDynamics(float dt_s)
{
    const float flight_start_s = behavior_config.synthetic_pad_warmup_s;
    const float burnout_s = flight_start_s + behavior_config.synthetic_burn_time_s;
    float drag_k;
    float drag_acceleration;

    synthetic_time_s += dt_s;
    telemetry.test_time_s = synthetic_time_s;
    telemetry.test_flight_time_s = synthetic_time_s - flight_start_s;

    synthetic_previous_velocity_m_s = telemetry.test_true_velocity_m_s;

    if (synthetic_time_s < flight_start_s)
    {
        telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_PAD;
        telemetry.test_flight_time_s = 0.0f;
        telemetry.test_true_altitude_m = behavior_config.initial_altitude_agl_m;
        telemetry.test_true_velocity_m_s = 0.0f;
        telemetry.test_true_acceleration_m_s2 = 0.0f;
        return;
    }

    if (synthetic_time_s < burnout_s)
    {
        telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_POWERED_ASCENT;
        telemetry.test_true_acceleration_m_s2 =
            behavior_config.synthetic_powered_accel_m_s2;
    }
    else
    {
        drag_k = behavior_config.synthetic_base_drag_k +
                 behavior_config.synthetic_airbrake_drag_k *
                 telemetry.controller_requested_deployment;

        drag_acceleration =
            -drag_k * telemetry.test_true_velocity_m_s *
            Behavior_Absolute(telemetry.test_true_velocity_m_s);

        telemetry.test_true_acceleration_m_s2 =
            -BEHAVIOR_GRAVITY_M_S2 + drag_acceleration;

        if (telemetry.test_true_velocity_m_s > 0.0f)
        {
            telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_COAST;
        }
        else
        {
            telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_DESCENT;
        }
    }

    telemetry.test_true_altitude_m +=
        telemetry.test_true_velocity_m_s * dt_s +
        0.5f * telemetry.test_true_acceleration_m_s2 * dt_s * dt_s;

    telemetry.test_true_velocity_m_s +=
        telemetry.test_true_acceleration_m_s2 * dt_s;

    if (telemetry.test_true_altitude_m > telemetry.true_apogee_m)
    {
        telemetry.true_apogee_m = telemetry.test_true_altitude_m;
    }

    if (!telemetry.apogee_reached &&
        (synthetic_previous_velocity_m_s > 0.0f) &&
        (telemetry.test_true_velocity_m_s <= 0.0f))
    {
        telemetry.apogee_reached = true;
        Behavior_FinalizeApogeeResult();
    }

    if ((telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_DESCENT) &&
        (telemetry.test_true_altitude_m <=
         behavior_config.initial_altitude_agl_m))
    {
        telemetry.test_true_altitude_m = behavior_config.initial_altitude_agl_m;
        telemetry.test_true_velocity_m_s = 0.0f;
        telemetry.test_true_acceleration_m_s2 = 0.0f;
        telemetry.flight_phase = BEHAVIOR_FLIGHT_PHASE_LANDED;
        telemetry.full_pipeline_complete = true;
        Behavior_FinalizeApogeeResult();
    }
}

static FusionVector Behavior_GetSyntheticBodyRate(void)
{
    FusionVector body_rate = FUSION_VECTOR_ZERO;

    if (telemetry.flight_phase == BEHAVIOR_FLIGHT_PHASE_PAD)
    {
        return body_rate;
    }

    body_rate.axis.x = 40.0f;
    body_rate.axis.y = 5.0f * sinf(0.7f * telemetry.test_flight_time_s);
    body_rate.axis.z = 3.0f * cosf(0.4f * telemetry.test_flight_time_s);
    return body_rate;
}

static void Behavior_UpdateTruthQuaternion(FusionVector body_rate_dps,
                                           float dt_s)
{
    const FusionVector half_rate = FusionVectorScale(
        body_rate_dps,
        FusionDegreesToRadians(0.5f));

    synthetic_truth_quaternion = FusionQuaternionAdd(
        synthetic_truth_quaternion,
        FusionQuaternionScale(
            FusionQuaternionVectorProduct(
                synthetic_truth_quaternion,
                half_rate),
            dt_s));

    synthetic_truth_quaternion = FusionQuaternionNormalise(
        synthetic_truth_quaternion);
}
// sync vectors
static FusionVector Behavior_RotateEarthToBody(FusionVector earth_vector,
                                               FusionQuaternion body_to_earth)
{
    const FusionMatrix rotation = FusionQuaternionToMatrix(body_to_earth);
    FusionVector body_vector;

    /* Multiply by the transpose of the body-to-Earth rotation matrix. */
    body_vector.axis.x =
        rotation.element.xx * earth_vector.axis.x +
        rotation.element.yx * earth_vector.axis.y +
        rotation.element.zx * earth_vector.axis.z;
    body_vector.axis.y =
        rotation.element.xy * earth_vector.axis.x +
        rotation.element.yy * earth_vector.axis.y +
        rotation.element.zy * earth_vector.axis.z;
    body_vector.axis.z =
        rotation.element.xz * earth_vector.axis.x +
        rotation.element.yz * earth_vector.axis.y +
        rotation.element.zz * earth_vector.axis.z;

    return body_vector;
}
// simulate sensors
static void Behavior_CreateSyntheticSensors(FusionVector body_rate_truth,
                                            FusionVector *gyroscope,
                                            FusionVector *accelerometer,
                                            FusionVector *magnetometer)
{
    FusionVector specific_force_earth = FUSION_VECTOR_ZERO;
    FusionVector magnetic_field_earth = FUSION_VECTOR_ZERO;

    if ((gyroscope == NULL) ||
        (accelerometer == NULL) ||
        (magnetometer == NULL))
    {
        return;
    }

    *gyroscope = body_rate_truth;
    gyroscope->axis.x += BEHAVIOR_SYNTH_GYRO_BIAS_DPS +
                         Behavior_Noise(BEHAVIOR_SYNTH_GYRO_NOISE_DPS);
    gyroscope->axis.y += Behavior_Noise(BEHAVIOR_SYNTH_GYRO_NOISE_DPS);
    gyroscope->axis.z += Behavior_Noise(BEHAVIOR_SYNTH_GYRO_NOISE_DPS);

    /*
     * Accelerometer specific force in the selected Earth convention.
     * The synthetic trajectory has only vertical translational acceleration.
     */
    if (behavior_config.fusion_convention == FusionConventionNed)
    {
        specific_force_earth.axis.z =
            -1.0f -
            (telemetry.test_true_acceleration_m_s2 /
             BEHAVIOR_GRAVITY_M_S2);

        magnetic_field_earth.axis.x = 0.45f;
        magnetic_field_earth.axis.y = 0.00f;
        magnetic_field_earth.axis.z = 0.89f;
    }
    else
    {
        specific_force_earth.axis.z =
            1.0f +
            (telemetry.test_true_acceleration_m_s2 /
             BEHAVIOR_GRAVITY_M_S2);

        magnetic_field_earth.axis.x = 0.45f;
        magnetic_field_earth.axis.y = 0.00f;
        magnetic_field_earth.axis.z = -0.89f;
    }

    *accelerometer = Behavior_RotateEarthToBody(
        specific_force_earth,
        synthetic_truth_quaternion);
    *magnetometer = Behavior_RotateEarthToBody(
        magnetic_field_earth,
        synthetic_truth_quaternion);

    accelerometer->axis.x +=
        Behavior_Noise(BEHAVIOR_SYNTH_ACCEL_NOISE_M_S2) /
        BEHAVIOR_GRAVITY_M_S2;
    accelerometer->axis.y +=
        Behavior_Noise(BEHAVIOR_SYNTH_ACCEL_NOISE_M_S2) /
        BEHAVIOR_GRAVITY_M_S2;
    accelerometer->axis.z +=
        Behavior_Noise(BEHAVIOR_SYNTH_ACCEL_NOISE_M_S2) /
        BEHAVIOR_GRAVITY_M_S2;

    magnetometer->axis.x += Behavior_Noise(BEHAVIOR_SYNTH_MAG_NOISE);
    magnetometer->axis.y += Behavior_Noise(BEHAVIOR_SYNTH_MAG_NOISE);
    magnetometer->axis.z += Behavior_Noise(BEHAVIOR_SYNTH_MAG_NOISE);
}
// update telemetry
static void Behavior_UpdateSyntheticTruthTelemetry(FusionVector gyroscope,
                                                   FusionVector accelerometer,
                                                   FusionVector magnetometer)
{
    const FusionEuler truth_euler = FusionQuaternionToEuler(
        synthetic_truth_quaternion);

    telemetry.test_roll_deg = truth_euler.angle.roll;
    telemetry.test_pitch_deg = truth_euler.angle.pitch;
    telemetry.test_yaw_deg = truth_euler.angle.yaw;

    telemetry.test_gyro_dps[0] = gyroscope.axis.x;
    telemetry.test_gyro_dps[1] = gyroscope.axis.y;
    telemetry.test_gyro_dps[2] = gyroscope.axis.z;

    telemetry.test_accel_g[0] = accelerometer.axis.x;
    telemetry.test_accel_g[1] = accelerometer.axis.y;
    telemetry.test_accel_g[2] = accelerometer.axis.z;

    telemetry.test_mag[0] = magnetometer.axis.x;
    telemetry.test_mag[1] = magnetometer.axis.y;
    telemetry.test_mag[2] = magnetometer.axis.z;

    telemetry.gyro_dps[0] = gyroscope.axis.x;
    telemetry.gyro_dps[1] = gyroscope.axis.y;
    telemetry.gyro_dps[2] = gyroscope.axis.z;
    telemetry.accel_g[0] = accelerometer.axis.x;
    telemetry.accel_g[1] = accelerometer.axis.y;
    telemetry.accel_g[2] = accelerometer.axis.z;
    telemetry.mag[0] = magnetometer.axis.x;
    telemetry.mag[1] = magnetometer.axis.y;
    telemetry.mag[2] = magnetometer.axis.z;
}
// verify errors and tolerances
static void Behavior_UpdateTestMetrics(void)
{
    float absolute_error;

    telemetry.altitude_error_m =
        telemetry.ekf_altitude_m - telemetry.test_true_altitude_m;
    telemetry.velocity_error_m_s =
        telemetry.ekf_velocity_m_s - telemetry.test_true_velocity_m_s;

    absolute_error = Behavior_Absolute(telemetry.altitude_error_m);
    if (absolute_error > telemetry.max_altitude_error_m)
    {
        telemetry.max_altitude_error_m = absolute_error;
    }

    absolute_error = Behavior_Absolute(telemetry.velocity_error_m_s);
    if (absolute_error > telemetry.max_velocity_error_m_s)
    {
        telemetry.max_velocity_error_m_s = absolute_error;
    }

    if (telemetry.ekf_altitude_m > telemetry.estimated_apogee_m)
    {
        telemetry.estimated_apogee_m = telemetry.ekf_altitude_m;
    }

    telemetry.ekf_within_altitude_tolerance =
        telemetry.max_altitude_error_m <=
        behavior_config.altitude_tolerance_m;

    if (telemetry.mode == BEHAVIOR_MODE_TEST_FULL_PIPELINE)
    {
        telemetry.fusion_roll_error_deg = Behavior_AngleDifferenceDegrees(
            telemetry.fusion_roll_deg,
            telemetry.test_roll_deg);
        telemetry.fusion_pitch_error_deg = Behavior_AngleDifferenceDegrees(
            telemetry.fusion_pitch_deg,
            telemetry.test_pitch_deg);
        telemetry.fusion_yaw_error_deg = Behavior_AngleDifferenceDegrees(
            telemetry.fusion_yaw_deg,
            telemetry.test_yaw_deg);

        /* Ignore the Fusion startup/pad period when accumulating pass metrics. */
        if ((telemetry.flight_phase != BEHAVIOR_FLIGHT_PHASE_PAD) &&
            !telemetry.fusion_startup)
        {
            absolute_error = Behavior_Absolute(
                telemetry.fusion_roll_error_deg);
            if (absolute_error > telemetry.max_fusion_roll_error_deg)
            {
                telemetry.max_fusion_roll_error_deg = absolute_error;
            }

            absolute_error = Behavior_Absolute(
                telemetry.fusion_pitch_error_deg);
            if (absolute_error > telemetry.max_fusion_pitch_error_deg)
            {
                telemetry.max_fusion_pitch_error_deg = absolute_error;
            }

            absolute_error = Behavior_Absolute(
                telemetry.fusion_yaw_error_deg);
            if (absolute_error > telemetry.max_fusion_yaw_error_deg)
            {
                telemetry.max_fusion_yaw_error_deg = absolute_error;
            }
        }

        telemetry.fusion_within_tolerance =
            telemetry.fusion_data_valid &&
            (telemetry.max_fusion_roll_error_deg <=
             behavior_config.fusion_roll_tolerance_deg) &&
            (telemetry.max_fusion_pitch_error_deg <=
             behavior_config.fusion_pitch_tolerance_deg) &&
            (telemetry.max_fusion_yaw_error_deg <=
             behavior_config.fusion_yaw_tolerance_deg);
    }

    telemetry.full_pipeline_pass =
        telemetry.full_pipeline_complete &&
        telemetry.fusion_within_tolerance &&
        telemetry.ekf_within_altitude_tolerance &&
        telemetry.controller_output_valid &&
        telemetry.controller_apogee_within_tolerance;
}
// view results
static void Behavior_FinalizeApogeeResult(void)
{
    if (telemetry.apogee_reached)
    {
        /* The control requirement can be judged as soon as apogee occurs. */
        telemetry.full_pipeline_complete = true;
    }

    telemetry.true_apogee_error_m =
        telemetry.true_apogee_m - behavior_config.target_apogee_m;

    telemetry.controller_apogee_within_tolerance =
        Behavior_Absolute(telemetry.true_apogee_error_m) <=
        behavior_config.altitude_tolerance_m;

    telemetry.full_pipeline_pass =
        telemetry.full_pipeline_complete &&
        telemetry.fusion_within_tolerance &&
        telemetry.ekf_within_altitude_tolerance &&
        telemetry.controller_output_valid &&
        telemetry.controller_apogee_within_tolerance;
}

static float Behavior_GetUpdatePeriodSeconds(void)
{
    return 1.0f / behavior_config.estimator_rate_hz;
}

static uint32_t Behavior_GetUpdatePeriodMilliseconds(void)
{
    uint32_t period_ms = (uint32_t)(1000.0f /
                                    behavior_config.estimator_rate_hz +
                                    0.5f);
    return (period_ms == 0U) ? 1U : period_ms;
}

static float Behavior_ClampFloat(float value,
                                 float minimum,
                                 float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static float Behavior_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Behavior_AngleDifferenceDegrees(float estimate,
                                             float truth)
{
    float difference = estimate - truth;

    while (difference > 180.0f)
    {
        difference -= 360.0f;
    }

    while (difference < -180.0f)
    {
        difference += 360.0f;
    }

    return difference;
}

static float Behavior_Noise(float amplitude)
{
    int32_t centered;

    noise_state = (1664525U * noise_state) + 1013904223U;
    centered = (int32_t)(noise_state >> 16) - 32768;

    return ((float)centered / 32768.0f) * amplitude;
}

static bool Behavior_Finite(float value)
{
    return isfinite(value) != 0;
}

static float Behavior_EarthVerticalToPositiveUp(FusionVector earth_acceleration)
{
    if (behavior_config.fusion_convention == FusionConventionNed)
    {
        return -earth_acceleration.axis.z * BEHAVIOR_GRAVITY_M_S2;
    }

    return earth_acceleration.axis.z * BEHAVIOR_GRAVITY_M_S2;
}
