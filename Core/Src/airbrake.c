#include "airbrake.h"
#include "tmc5240.h"

#define ABS32(x) ((x) < 0 ? -(x) : (x))

/* Keep these masks local so the airbrake layer can thermal-throttle on OTPW
 * without calling TMC5240_CheckSafety(), which currently treats OTPW as fatal. */
#define AIRBRAKE_TMC_SHORT_MASK \
    (TMC5240_DRV_S2GB | TMC5240_DRV_S2GA | TMC5240_DRV_S2VSB | TMC5240_DRV_S2VSA)

#define AIRBRAKE_TMC_GSTAT_MASK \
    (TMC5240_GSTAT_DRV_ERR | TMC5240_GSTAT_UV_CP | \
     TMC5240_GSTAT_REGISTER_RESET | TMC5240_GSTAT_VM_UVLO)

static Airbrake_Config_t ab_cfg;
static Airbrake_Telemetry_t ab;
static float ab_prev_accel_g = 0.0f;
static uint32_t ab_burnout_time_ms = 0;
static uint32_t ab_move_start_ms = 0;

static uint8_t clamp_percent(uint8_t percent)
{
    return (percent > 100U) ? 100U : percent;
}

static int32_t percent_to_counts(uint8_t percent)
{
    int64_t span = (int64_t)ab_cfg.open_counts - (int64_t)ab_cfg.closed_counts;
    int64_t pos = (int64_t)ab_cfg.closed_counts + ((span * (int64_t)percent) / 100LL);
    return (int32_t)pos;
}

static uint8_t counts_to_percent(int32_t counts)
{
    int64_t span = (int64_t)ab_cfg.open_counts - (int64_t)ab_cfg.closed_counts;
    int64_t rel = (int64_t)counts - (int64_t)ab_cfg.closed_counts;
    int64_t pct;

    if (span == 0)
    {
        return 0U;
    }

    pct = (rel * 100LL) / span;
    if (pct < 0LL)
    {
        return 0U;
    }
    if (pct > 100LL)
    {
        return 100U;
    }
    return (uint8_t)pct;
}

static HAL_StatusTypeDef apply_motion_profile(bool thermal_limited)
{
    HAL_StatusTypeDef status;
    uint32_t vmax = thermal_limited ? ab_cfg.vmax_thermal : ab_cfg.vmax_normal;
    uint32_t amax = thermal_limited ? ab_cfg.amax_thermal : ab_cfg.amax_normal;
    uint32_t dmax = thermal_limited ? ab_cfg.dmax_thermal : ab_cfg.dmax_normal;
    uint8_t ihold = thermal_limited ? ab_cfg.ihold_hold : ab_cfg.ihold_run;
    uint8_t irun = thermal_limited ? ab_cfg.irun_hold : ab_cfg.irun_run;
    uint8_t scaler = thermal_limited ? ab_cfg.scaler_hold : ab_cfg.scaler_run;

    status = TMC5240_SetCurrent(ihold, irun);
    if (status != HAL_OK) return status;

    status = TMC5240_SetGlobalScaler(scaler);
    if (status != HAL_OK) return status;

    status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE, TMC5240_RAMPMODE_POSITION);
    if (status != HAL_OK) return status;

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, vmax);
    if (status != HAL_OK) return status;

    status = TMC5240_WriteReg(TMC5240_REG_AMAX, amax);
    if (status != HAL_OK) return status;

    status = TMC5240_WriteReg(TMC5240_REG_DMAX, dmax);
    if (status != HAL_OK) return status;

    status = TMC5240_WriteReg(TMC5240_REG_XTARGET, (uint32_t)ab.target_counts);
    return status;
}

static HAL_StatusTypeDef set_hold_current(void)
{
    HAL_StatusTypeDef status;

    status = TMC5240_SetCurrent(ab_cfg.ihold_hold, ab_cfg.irun_hold);
    if (status != HAL_OK) return status;

    return TMC5240_SetGlobalScaler(ab_cfg.scaler_hold);
}

static void enter_fault(uint32_t fault_bits)
{
    ab.faults |= fault_bits;
    ab.state = AIRBRAKE_STATE_FAULT;
    (void)TMC5240_StopVelocity();
    TMC5240_DisableDriver();
}

static HAL_StatusTypeDef read_driver_status(void)
{
    uint32_t drv = 0;
    uint32_t gs = 0;

    if (TMC5240_GetDRV_STATUS(&drv) != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_SPI | AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if (TMC5240_GetGSTAT(&gs) != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_SPI | AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    ab.last_drv_status = drv;
    ab.last_gstat = gs;

    if ((drv & AIRBRAKE_TMC_SHORT_MASK) != 0U)
    {
        enter_fault(AIRBRAKE_FAULT_SHORT | AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((drv & TMC5240_DRV_OT) != 0U)
    {
        enter_fault(AIRBRAKE_FAULT_OVERTEMP | AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((gs & TMC5240_GSTAT_VM_UVLO) != 0U)
    {
        enter_fault(AIRBRAKE_FAULT_UNDERVOLTAGE | AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((gs & AIRBRAKE_TMC_GSTAT_MASK) != 0U)
    {
        enter_fault(AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if ((drv & TMC5240_DRV_OTPW) != 0U)
    {
        ab.thermal_limited = true;
    }

    return HAL_OK;
}

void Airbrake_DefaultConfig(Airbrake_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->closed_counts = 0;
    cfg->open_counts = 20000;
    cfg->position_tolerance = 100;

    cfg->vmax_normal = 20000;
    cfg->amax_normal = 1000;
    cfg->dmax_normal = 1000;

    cfg->vmax_thermal = 8000;
    cfg->amax_thermal = 300;
    cfg->dmax_thermal = 300;

    cfg->ihold_run = 4;
    cfg->irun_run = 12;
    cfg->scaler_run = 64;

    cfg->ihold_hold = 2;
    cfg->irun_hold = 4;
    cfg->scaler_hold = 32;

    cfg->launch_accel_g = 3.0f;
    cfg->burnout_accel_g = 1.5f;
    cfg->burnout_drop_g = 2.0f;
    cfg->post_burnout_delay_ms = 1000U;

    cfg->move_timeout_ms = 2000U;
}

HAL_StatusTypeDef Airbrake_Init(const Airbrake_Config_t *cfg)
{
    if (cfg == NULL || cfg->closed_counts == cfg->open_counts ||
        cfg->position_tolerance <= 0 || cfg->post_burnout_delay_ms == 0U)
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
    ab.current_percent = 0;
    ab.target_percent = 0;
    ab.calibrated = true;
    ab.launched = false;
    ab.burnout_detected = false;
    ab.deployment_allowed = false;
    ab.manual_override = false;
    ab.thermal_limited = false;
    ab.last_drv_status = 0;
    ab.last_gstat = 0;
    ab_prev_accel_g = 0.0f;
    ab_burnout_time_ms = 0U;
    ab_move_start_ms = 0U;

    if (TMC5240_InitBasic() != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_TMC_STATUS);
        return HAL_ERROR;
    }

    if (TMC5240_WriteReg(TMC5240_REG_XACTUAL, (uint32_t)ab_cfg.closed_counts) != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_SPI);
        return HAL_ERROR;
    }

    return Airbrake_Retract();
}

HAL_StatusTypeDef Airbrake_CalibrateBounds(int32_t closed_counts, int32_t open_counts, bool zero_current_as_closed)
{
    if (closed_counts == open_counts)
    {
        enter_fault(AIRBRAKE_FAULT_BAD_CONFIG);
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
            enter_fault(AIRBRAKE_FAULT_SPI);
            return HAL_ERROR;
        }
    }

    ab.target_percent = 0U;
    ab.target_counts = ab_cfg.closed_counts;
    return Airbrake_Retract();
}

void Airbrake_SetManualOverride(bool enabled)
{
    ab.manual_override = enabled;
}

void Airbrake_UpdateFlightGate(float accel_g, uint32_t now_ms)
{
    if (!ab.launched && accel_g >= ab_cfg.launch_accel_g)
    {
        ab.launched = true;
    }

    if (ab.launched && !ab.burnout_detected)
    {
        bool sudden_drop = ((ab_prev_accel_g - accel_g) >= ab_cfg.burnout_drop_g);
        bool below_burnout = (accel_g <= ab_cfg.burnout_accel_g);

        if (sudden_drop && below_burnout)
        {
            ab.burnout_detected = true;
            ab_burnout_time_ms = now_ms;
        }
    }

    if (ab.burnout_detected && !ab.deployment_allowed)
    {
        if ((now_ms - ab_burnout_time_ms) >= ab_cfg.post_burnout_delay_ms)
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

bool Airbrake_IsBusy(void)
{
    return (ab.state == AIRBRAKE_STATE_MOVING);
}

bool Airbrake_IsDeploymentAllowed(void)
{
    return ab.manual_override || ab.deployment_allowed;
}

HAL_StatusTypeDef Airbrake_SetTargetPercent(uint8_t percent)
{
    uint8_t p = clamp_percent(percent);
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

    /* Retraction to 0% is always allowed. Extension is locked until burnout + delay. */
    if (p > ab.current_percent && !Airbrake_IsDeploymentAllowed())
    {
        ab.faults |= AIRBRAKE_FAULT_NOT_ALLOWED;
        return HAL_BUSY;
    }

    if (read_driver_status() != HAL_OK)
    {
        return HAL_ERROR;
    }

    ab.target_percent = p;
    ab.target_counts = percent_to_counts(p);
    ab_move_start_ms = HAL_GetTick();
    ab.state = AIRBRAKE_STATE_MOVING;

    status = apply_motion_profile(false);
    if (status != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_SPI);
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Airbrake_Retract(void)
{
    return Airbrake_SetTargetPercent(0U);
}

HAL_StatusTypeDef Airbrake_DeployFull(void)
{
    return Airbrake_SetTargetPercent(100U);
}

HAL_StatusTypeDef Airbrake_Update(uint32_t now_ms)
{
    int32_t x = 0;

    if (ab.state == AIRBRAKE_STATE_UNINIT || ab.state == AIRBRAKE_STATE_FAULT)
    {
        return HAL_ERROR;
    }

    if (read_driver_status() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (TMC5240_GetXACTUAL(&x) != HAL_OK)
    {
        enter_fault(AIRBRAKE_FAULT_SPI);
        return HAL_ERROR;
    }

    ab.xactual = x;
    ab.current_percent = counts_to_percent(x);

    if (ab.state == AIRBRAKE_STATE_MOVING)
    {
        if (ab.thermal_limited)
        {
            /* Reapply same target with reduced speed/current. */
            if (apply_motion_profile(true) != HAL_OK)
            {
                enter_fault(AIRBRAKE_FAULT_SPI);
                return HAL_ERROR;
            }
        }

        if (ABS32(ab.xactual - ab.target_counts) <= ab_cfg.position_tolerance)
        {
            if (set_hold_current() != HAL_OK)
            {
                enter_fault(AIRBRAKE_FAULT_SPI);
                return HAL_ERROR;
            }
            ab.current_percent = ab.target_percent;
            ab.state = AIRBRAKE_STATE_HOLDING;
        }
        else if ((now_ms - ab_move_start_ms) > ab_cfg.move_timeout_ms)
        {
            enter_fault(AIRBRAKE_FAULT_MOVE_TIMEOUT);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

void Airbrake_EStop(void)
{
    enter_fault(AIRBRAKE_FAULT_TMC_STATUS);
}

Airbrake_Telemetry_t Airbrake_GetTelemetry(void)
{
    return ab;
}
