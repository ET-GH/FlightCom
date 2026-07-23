/**
 * @file airbrake.c
 * @brief Airbrake actuator control, flight gating, and TMC5240 fault handling.
 *
 * This module converts requested deployment percentages into motor positions,
 * prevents extension before the configured post-burnout gate, supervises the
 * TMC5240 driver, and records actuator state in Airbrake_Telemetry_t.
 *
 * Retraction is always permitted. Extension requires either the automatic
 * launch/burnout gate or an explicit manual override.
 */

#include "airbrake.h"

#include <stddef.h>

#include "tmc5240.h"

/* -------------------------------------------------------------------------- */
/* Private constants                                                          */
/* -------------------------------------------------------------------------- */

#define ABS32(x) ((x) < 0 ? -(x) : (x))

/**
 * Driver-status faults handled directly by this layer.
 *
 * OTPW is intentionally excluded because it is treated as a recoverable
 * thermal-warning condition that reduces motor speed and current.
 */
#define AIRBRAKE_TMC_SHORT_MASK \
    (TMC5240_DRV_S2GB | TMC5240_DRV_S2GA | \
     TMC5240_DRV_S2VSB | TMC5240_DRV_S2VSA)

#define AIRBRAKE_TMC_GSTAT_MASK \
    (TMC5240_GSTAT_DRV_ERR | TMC5240_GSTAT_UV_CP | \
     TMC5240_GSTAT_REGISTER_RESET | TMC5240_GSTAT_VM_UVLO)

/* -------------------------------------------------------------------------- */
/* Module state                                                               */
/* -------------------------------------------------------------------------- */

/** Active actuator configuration copied during Airbrake_Init(). */
static Airbrake_Config_t ab_cfg;

/** Current airbrake state, position, gate state, and fault telemetry. */
static Airbrake_Telemetry_t ab;

/** Previous acceleration sample used to detect the burnout acceleration drop. */
static float ab_prev_accel_g = 0.0f;

/** Timestamp of the detected burnout event. */
static uint32_t ab_burnout_time_ms = 0U;

/** Timestamp at which the current position command began. */
static uint32_t ab_move_start_ms = 0U;

/*
 * True while a relative motor-step command owns the actuator.
 *
 * Percentage commands, including automatic controller commands, are rejected
 * until the relative movement reaches its target or faults.
 */
static bool ab_manual_step_active = false;

/* -------------------------------------------------------------------------- */
/* Private function declarations                                              */
/* -------------------------------------------------------------------------- */

static int32_t Airbrake_PercentToCounts(uint8_t percent);
static uint8_t Airbrake_CountsToPercent(int32_t counts);
static HAL_StatusTypeDef Airbrake_ApplyMotionProfile(bool thermal_limited);
static void Airbrake_EnterFault(uint32_t fault_bits);
static HAL_StatusTypeDef Airbrake_ReadDriverStatus(void);

/* ========================================================================== */
/* Public configuration and initialization                                    */
/* ========================================================================== */

/**
 * @brief Populate an airbrake configuration with conservative defaults.
 *
 * @param cfg Configuration structure to initialize. A NULL pointer is ignored.
 */
void Airbrake_DefaultConfig(Airbrake_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    /* Mechanical position calibration. */
    cfg->closed_counts = 0;
    cfg->open_counts = 20000;
    cfg->position_tolerance = 100;

    /* Normal position-mode limits. */
    cfg->vmax_normal = 20000;
    cfg->amax_normal = 1000;
    cfg->dmax_normal = 1000;

    /* Reduced limits used after an overtemperature pre-warning. */
    cfg->vmax_thermal = 8000;
    cfg->amax_thermal = 300;
    cfg->dmax_thermal = 300;

    /* Current settings while moving. */
    cfg->ihold_run = 4;
    cfg->irun_run = 12;
    cfg->scaler_run = 64;

    /* Reduced current settings while holding position. */
    cfg->ihold_hold = 2;
    cfg->irun_hold = 4;
    cfg->scaler_hold = 32;

    /* Launch and burnout deployment gate thresholds. */
    cfg->launch_accel_g = 3.0f;
    cfg->burnout_accel_g = 1.5f;
    cfg->burnout_drop_g = 2.0f;
    cfg->post_burnout_delay_ms = 1000U;

    cfg->move_timeout_ms = 2000U;
}

/**
 * @brief Initialize the actuator state and TMC5240 driver.
 *
 * The current motor position is defined as the configured closed position and
 * an initial retraction command is issued. The airbrake remains locked against
 * extension until the flight gate opens or manual override is enabled.
 *
 * @param cfg Validated actuator configuration.
 * @return HAL_OK when the driver is initialized and the retract command is
 *         accepted; otherwise an error status.
 */
HAL_StatusTypeDef Airbrake_Init(const Airbrake_Config_t *cfg)
{
    if ((cfg == NULL) ||
        (cfg->closed_counts == cfg->open_counts) ||
        (cfg->position_tolerance <= 0) ||
        (cfg->post_burnout_delay_ms == 0U))
    {
        ab.state = AIRBRAKE_STATE_FAULT;
        ab.faults = AIRBRAKE_FAULT_BAD_CONFIG;
        return HAL_ERROR;
    }

    ab_cfg = *cfg;

    ab.state = AIRBRAKE_STATE_LOCKED;
    ab.faults = AIRBRAKE_FAULT_NONE;
    ab.xactual = 0;
    ab.target_counts = ab_cfg.closed_counts;
    ab.current_percent = 0U;
    ab.target_percent = 0U;
    ab.calibrated = true;
    ab.launched = false;
    ab.burnout_detected = false;
    ab.deployment_allowed = false;
    ab.manual_override = false;
    ab.thermal_limited = false;
    ab.last_drv_status = 0U;
    ab.last_gstat = 0U;

    ab_prev_accel_g = 0.0f;
    ab_burnout_time_ms = 0U;
    ab_move_start_ms = 0U;

    ab_manual_step_active = false;

    if (TMC5240_InitBasic() != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    /* Establish the software position reference before commanding retraction. */
    if (TMC5240_WriteReg(TMC5240_REG_XACTUAL,
                         (uint32_t)ab_cfg.closed_counts) != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
        return HAL_ERROR;
    }

    return Airbrake_Retract();
}

/**
 * @brief Update the closed and open motor-position calibration values.
 *
 * @param closed_counts Motor count corresponding to 0% deployment.
 * @param open_counts Motor count corresponding to 100% deployment.
 * @param zero_current_as_closed When true, redefine the current closed count as
 *        zero and shift the open count by the same amount.
 * @return HAL_OK when calibration and the following retract command succeed.
 */
HAL_StatusTypeDef Airbrake_CalibrateBounds(int32_t closed_counts,
                                           int32_t open_counts,
                                           bool zero_current_as_closed)
{
    if (closed_counts == open_counts)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_BAD_CONFIG);
        return HAL_ERROR;
    }

    ab_cfg.closed_counts = closed_counts;
    ab_cfg.open_counts = open_counts;
    ab.calibrated = true;

    if (zero_current_as_closed)
    {
        ab_cfg.closed_counts = 0;
        ab_cfg.open_counts = open_counts - closed_counts;

        if (TMC5240_WriteReg(TMC5240_REG_XACTUAL, 0U) != HAL_OK)
        {
            Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
            return HAL_ERROR;
        }
    }

    ab.target_percent = 0U;
    ab.target_counts = ab_cfg.closed_counts;

    return Airbrake_Retract();
}

/* ========================================================================== */
/* Flight safety gate                                                         */
/* ========================================================================== */

/**
 * @brief Enable or disable the operator-controlled deployment override.
 *
 * Manual override bypasses only the launch/burnout deployment gate. It does not
 * bypass calibration checks, driver faults, or TMC5240 safety supervision.
 */
void Airbrake_SetManualOverride(bool enabled)
{
    ab.manual_override = enabled;
}

/**
 * @brief Update launch, burnout, and post-burnout deployment-gate detection.
 *
 * Launch is latched when acceleration exceeds launch_accel_g. Burnout is then
 * latched when acceleration both drops by burnout_drop_g and falls below
 * burnout_accel_g. Extension becomes available after post_burnout_delay_ms.
 *
 * @param accel_g Current acceleration measurement in g.
 * @param now_ms Current system timestamp in milliseconds.
 */
void Airbrake_UpdateFlightGate(float accel_g, uint32_t now_ms)
{
    if (!ab.launched && (accel_g >= ab_cfg.launch_accel_g))
    {
        ab.launched = true;
    }

    if (ab.launched && !ab.burnout_detected)
    {
        const bool sudden_drop =
            ((ab_prev_accel_g - accel_g) >= ab_cfg.burnout_drop_g);
        const bool below_burnout = (accel_g <= ab_cfg.burnout_accel_g);

        if (sudden_drop && below_burnout)
        {
            ab.burnout_detected = true;
            ab_burnout_time_ms = now_ms;
        }
    }

    if (ab.burnout_detected && !ab.deployment_allowed)
    {
        if ((uint32_t)(now_ms - ab_burnout_time_ms) >=
            ab_cfg.post_burnout_delay_ms)
        {
            ab.deployment_allowed = true;

            if (ab.state == AIRBRAKE_STATE_LOCKED)
            {
                ab.state = AIRBRAKE_STATE_READY;
            }
        }
    }

    ab_prev_accel_g = accel_g;
}

/** @return true while the actuator is moving toward a target position. */
bool Airbrake_IsBusy(void)
{
    return (ab.state == AIRBRAKE_STATE_MOVING);
}

/**
 * @return true while a relative motor-step command owns the actuator.
 */
bool Airbrake_IsManualStepActive(void)
{
    return ab_manual_step_active;
}

/**
 * @return true when extension is allowed by the flight gate or manual override.
 */
bool Airbrake_IsDeploymentAllowed(void)
{
    return ab.manual_override || ab.deployment_allowed;
}

/* ========================================================================== */
/* Position commands                                                          */
/* ========================================================================== */

/**
 * @brief Command an airbrake deployment position from 0% through 100%.
 *
 * Values above 100% are saturated. Retraction is always permitted; extension
 * beyond the current position is rejected until deployment is allowed.
 *
 * @param percent Requested deployment percentage.
 * @return HAL_OK when the position command is accepted, HAL_BUSY when extension
 *         is blocked by the flight gate, or an error status on a fault.
 */
HAL_StatusTypeDef Airbrake_SetTargetPercent(uint8_t percent)
{
    const uint8_t requested_percent = (percent > 100U) ? 100U : percent;
    HAL_StatusTypeDef status;

    if (!ab.calibrated)
    {
        ab.faults |= AIRBRAKE_FAULT_NOT_CALIBRATED;
        return HAL_ERROR;
    }

    if (ab.state == AIRBRAKE_STATE_FAULT)
    {
        return HAL_ERROR;
    }

    /* Retraction is always allowed. Only outward motion uses the flight gate. */
    if ((requested_percent > ab.current_percent) &&
        !Airbrake_IsDeploymentAllowed())
    {
        ab.faults |= AIRBRAKE_FAULT_NOT_ALLOWED;
        return HAL_BUSY;
    }

    if (Airbrake_ReadDriverStatus() != HAL_OK)
    {
        return HAL_ERROR;
    }

    ab.target_percent = requested_percent;
    ab.target_counts = Airbrake_PercentToCounts(requested_percent);
    ab_move_start_ms = HAL_GetTick();
    ab.state = AIRBRAKE_STATE_MOVING;

    status = Airbrake_ApplyMotionProfile(false);
    if (status != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
        return status;
    }

    /*
     * A relative movement has exclusive ownership. This rejects:
     *
     * 1. Radio percentage commands.
     * 2. Automatic controller deployment commands.
     * 3. Controller-disabled automatic retraction commands.
     */
    if (ab_manual_step_active)
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
 * @brief Move the airbrake motor by a signed relative number of steps.
 *
 * Positive values always mean physical deployment and negative values always
 * mean physical retraction. The calibrated open/closed count direction is used
 * to convert that physical meaning into the correct motor-count direction.
 *
 * This function intentionally does not clamp the target to the calibrated
 * mechanical open/closed positions. It only prevents signed 32-bit overflow.
 *
 * An existing normal percentage movement may be replaced by this command.
 * Another relative movement is rejected until the first one completes.
 */
HAL_StatusTypeDef Airbrake_MoveRelativeSteps(int16_t steps)
{
    const int32_t deployment_direction =
        (ab_cfg.open_counts >= ab_cfg.closed_counts) ? 1 : -1;

    int32_t xactual = 0;
    int64_t target;
    HAL_StatusTypeDef status;

    if (!ab.calibrated)
    {
        ab.faults |= AIRBRAKE_FAULT_NOT_CALIBRATED;
        return HAL_ERROR;
    }

    if (ab.state == AIRBRAKE_STATE_FAULT)
    {
        return HAL_ERROR;
    }

    if (ab_manual_step_active)
    {
        return HAL_BUSY;
    }

    if (Airbrake_ReadDriverStatus() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (TMC5240_GetXACTUAL(&xactual) != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
        return HAL_ERROR;
    }

    /*
     * Positive protocol steps always deploy, even when deployment corresponds
     * to decreasing motor counts.
     */
    target =
        (int64_t)xactual +
        ((int64_t)steps * (int64_t)deployment_direction);

    /*
     * No mechanical bounds are applied. Only protect the TMC5240 signed
     * position representation from arithmetic overflow.
     */
    if ((target > (int64_t)INT32_MAX) ||
        (target < (int64_t)INT32_MIN))
    {
        return HAL_ERROR;
    }

    ab.xactual = xactual;
    ab.target_counts = (int32_t)target;
    ab.target_percent = Airbrake_CountsToPercent(ab.target_counts);
    ab_move_start_ms = HAL_GetTick();

    /*
     * A zero-step request is already complete. The bridge will produce the
     * completion ACK during its next completion poll.
     */
    if (steps == 0)
    {
        ab.state = AIRBRAKE_STATE_HOLDING;
        ab_manual_step_active = false;
        return HAL_OK;
    }

    ab_manual_step_active = true;
    ab.state = AIRBRAKE_STATE_MOVING;

    /*
     * Reuse the existing configured actuator motion profile and preserve
     * thermal limiting when it is already active.
     */
    status = Airbrake_ApplyMotionProfile(ab.thermal_limited);
    if (status != HAL_OK)
    {
        ab_manual_step_active = false;
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
        return status;
    }

    return HAL_OK;
}

/** @brief Command the fully retracted 0% position. */
HAL_StatusTypeDef Airbrake_Retract(void)
{
    return Airbrake_SetTargetPercent(0U);
}

/** @brief Command the fully deployed 100% position. */
HAL_StatusTypeDef Airbrake_DeployFull(void)
{
    return Airbrake_SetTargetPercent(100U);
}

/* ========================================================================== */
/* Periodic actuator service and telemetry                                    */
/* ========================================================================== */

/**
 * @brief Service driver safety, position feedback, thermal limiting, and timeout.
 *
 * Call this function periodically. While moving, it reduces the motion profile
 * after OTPW, switches to hold current at the target, and enters a fault if the
 * configured movement timeout expires.
 *
 * @param now_ms Current system timestamp in milliseconds.
 * @return HAL_OK during normal operation, HAL_TIMEOUT on movement timeout, or
 *         HAL_ERROR when the module or motor driver is faulted.
 */
HAL_StatusTypeDef Airbrake_Update(uint32_t now_ms)
{
    int32_t xactual = 0;

    if ((ab.state == AIRBRAKE_STATE_UNINIT) ||
        (ab.state == AIRBRAKE_STATE_FAULT))
    {
        return HAL_ERROR;
    }

    if (Airbrake_ReadDriverStatus() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (TMC5240_GetXACTUAL(&xactual) != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
        return HAL_ERROR;
    }

    ab.xactual = xactual;
    ab.current_percent = Airbrake_CountsToPercent(xactual);

    if (ab.state == AIRBRAKE_STATE_MOVING)
    {
        if (ab.thermal_limited)
        {
            /* Keep the same target, but replace normal limits with the reduced
             * thermal-warning speed, acceleration, and current profile. */
            if (Airbrake_ApplyMotionProfile(true) != HAL_OK)
            {
                Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
                return HAL_ERROR;
            }
        }

        if (ABS32(ab.xactual - ab.target_counts) <=
            ab_cfg.position_tolerance)
        {
            /* The original single-use hold-current helper is integrated here so
             * the target-reached transition is visible in one place. */
            if (TMC5240_SetCurrent(ab_cfg.ihold_hold,
                                   ab_cfg.irun_hold) != HAL_OK)
            {
                Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
                return HAL_ERROR;
            }

            if (TMC5240_SetGlobalScaler(ab_cfg.scaler_hold) != HAL_OK)
            {
                Airbrake_EnterFault(AIRBRAKE_FAULT_SPI);
                return HAL_ERROR;
            }

            ab.current_percent = ab.target_percent;
            ab.state = AIRBRAKE_STATE_HOLDING;
            ab_manual_step_active = false;
        }
        else if ((uint32_t)(now_ms - ab_move_start_ms) >
                 ab_cfg.move_timeout_ms)
        {
            Airbrake_EnterFault(AIRBRAKE_FAULT_MOVE_TIMEOUT);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/**
 * @brief Immediately stop motion, disable the motor driver, and latch a fault.
 */
void Airbrake_EStop(void)
{
    Airbrake_EnterFault(AIRBRAKE_FAULT_TMC_STATUS);
}

/** @return A snapshot of the current airbrake telemetry structure. */
Airbrake_Telemetry_t Airbrake_GetTelemetry(void)
{
    return ab;
}

/**
 * @brief Clear recoverable airbrake and TMC5240 fault latches in place.
 *
 * This does not reset:
 * - motor position,
 * - target position,
 * - calibration,
 * - launch or burnout state,
 * - deployment permission,
 * - manual override,
 * - Behavior,
 * - EKF,
 * - controller state,
 * - flight-computer state.
 *
 * If the physical hardware fault remains active, this function fails and the
 * actuator remains faulted.
 */
HAL_StatusTypeDef Airbrake_ClearFaults(void)
{
    uint32_t drv_status = 0U;
    uint32_t gstat = 0U;
    const Airbrake_State_t previous_state = ab.state;

    /*
     * Configuration errors cannot safely be cleared without correcting the
     * actual configuration.
     */
    if (!ab.calibrated ||
        (ab_cfg.closed_counts == ab_cfg.open_counts))
    {
        return HAL_ERROR;
    }

    /*
     * GSTAT uses write-one-to-clear bits. This is the same mask already used
     * during TMC5240 initialization.
     */
    if (TMC5240_WriteReg(TMC5240_REG_GSTAT, 0x0000001FU) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Read the actual hardware state after clearing the stored status bits.
     */
    if ((TMC5240_GetDRV_STATUS(&drv_status) != HAL_OK) ||
        (TMC5240_GetGSTAT(&gstat) != HAL_OK))
    {
        return HAL_ERROR;
    }

    ab.last_drv_status = drv_status;
    ab.last_gstat = gstat;

    /*
     * Do not falsely report success while a physical short, overtemperature,
     * undervoltage, driver error, or charge-pump fault is still present.
     */
    if (((drv_status & AIRBRAKE_TMC_SHORT_MASK) != 0U) ||
        ((drv_status & TMC5240_DRV_OT) != 0U) ||
        ((gstat & AIRBRAKE_TMC_GSTAT_MASK) != 0U))
    {
        return HAL_ERROR;
    }

    ab.faults = AIRBRAKE_FAULT_NONE;
    ab.thermal_limited =
        ((drv_status & TMC5240_DRV_OTPW) != 0U);

    /*
     * Re-enable the bridge only if an actuator fault had disabled it.
     * Do not restart the previous movement.
     */
    if (previous_state == AIRBRAKE_STATE_FAULT)
    {
        if (TMC5240_EnableDriver() != HAL_OK)
        {
            Airbrake_EnterFault(AIRBRAKE_FAULT_TMC_STATUS);
            return HAL_ERROR;
        }

        ab.state = Airbrake_IsDeploymentAllowed()
            ? AIRBRAKE_STATE_READY
            : AIRBRAKE_STATE_LOCKED;
    }

    return HAL_OK;
}

/* ========================================================================== */
/* Private position conversion helpers                                        */
/* ========================================================================== */

/** Convert a deployment percentage to an absolute TMC5240 position count. */
static int32_t Airbrake_PercentToCounts(uint8_t percent)
{
    const int64_t span =
        (int64_t)ab_cfg.open_counts - (int64_t)ab_cfg.closed_counts;
    const int64_t position =
        (int64_t)ab_cfg.closed_counts +
        ((span * (int64_t)percent) / 100LL);

    return (int32_t)position;
}

/** Convert an absolute motor position count to a saturated percentage. */
static uint8_t Airbrake_CountsToPercent(int32_t counts)
{
    const int64_t span =
        (int64_t)ab_cfg.open_counts - (int64_t)ab_cfg.closed_counts;
    const int64_t relative =
        (int64_t)counts - (int64_t)ab_cfg.closed_counts;
    int64_t percent;

    if (span == 0LL)
    {
        return 0U;
    }

    percent = (relative * 100LL) / span;

    if (percent < 0LL)
    {
        return 0U;
    }

    if (percent > 100LL)
    {
        return 100U;
    }

    return (uint8_t)percent;
}

/* ========================================================================== */
/* Private TMC5240 control and fault handling                                  */
/* ========================================================================== */

/**
 * Apply either the normal or thermal-limited position motion profile and issue
 * the currently stored target position.
 */
static HAL_StatusTypeDef Airbrake_ApplyMotionProfile(bool thermal_limited)
{
    const uint32_t vmax =
        thermal_limited ? ab_cfg.vmax_thermal : ab_cfg.vmax_normal;
    const uint32_t amax =
        thermal_limited ? ab_cfg.amax_thermal : ab_cfg.amax_normal;
    const uint32_t dmax =
        thermal_limited ? ab_cfg.dmax_thermal : ab_cfg.dmax_normal;
    const uint8_t ihold =
        thermal_limited ? ab_cfg.ihold_hold : ab_cfg.ihold_run;
    const uint8_t irun =
        thermal_limited ? ab_cfg.irun_hold : ab_cfg.irun_run;
    const uint8_t scaler =
        thermal_limited ? ab_cfg.scaler_hold : ab_cfg.scaler_run;
    HAL_StatusTypeDef status;

    status = TMC5240_SetCurrent(ihold, irun);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_SetGlobalScaler(scaler);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE,
                              TMC5240_RAMPMODE_POSITION);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, vmax);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_AMAX, amax);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_DMAX, dmax);
    if (status != HAL_OK)
    {
        return status;
    }

    return TMC5240_WriteReg(TMC5240_REG_XTARGET,
                            (uint32_t)ab.target_counts);
}

/** Latch the supplied fault bits and force the motor driver into a safe state. */
static void Airbrake_EnterFault(uint32_t fault_bits)
{
    ab.faults |= fault_bits;
    ab.state = AIRBRAKE_STATE_FAULT;
    ab_manual_step_active = false;

    (void)TMC5240_StopVelocity();
    TMC5240_DisableDriver();
}

/**
 * Read and classify TMC5240 status registers.
 *
 * Hard faults enter AIRBRAKE_STATE_FAULT. OTPW instead latches thermal limiting
 * so the current movement can continue with the reduced profile.
 */
static HAL_StatusTypeDef Airbrake_ReadDriverStatus(void)
{
    uint32_t drv_status = 0U;
    uint32_t global_status = 0U;

    if (TMC5240_GetDRV_STATUS(&drv_status) != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI |
                            AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if (TMC5240_GetGSTAT(&global_status) != HAL_OK)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SPI |
                            AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    ab.last_drv_status = drv_status;
    ab.last_gstat = global_status;

    if ((drv_status & AIRBRAKE_TMC_SHORT_MASK) != 0U)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_SHORT |
                            AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((drv_status & TMC5240_DRV_OT) != 0U)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_OVERTEMP |
                            AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((global_status & TMC5240_GSTAT_VM_UVLO) != 0U)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_UNDERVOLTAGE |
                            AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((global_status & AIRBRAKE_TMC_GSTAT_MASK) != 0U)
    {
        Airbrake_EnterFault(AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((drv_status & TMC5240_DRV_OTPW) != 0U)
    {
        ab.thermal_limited = true;
    }

    return HAL_OK;
}
