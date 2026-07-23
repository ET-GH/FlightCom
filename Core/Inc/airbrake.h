#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    AIRBRAKE_STATE_UNINIT = 0,
    AIRBRAKE_STATE_LOCKED,
    AIRBRAKE_STATE_READY,
    AIRBRAKE_STATE_MOVING,
    AIRBRAKE_STATE_HOLDING,
    AIRBRAKE_STATE_FAULT
} Airbrake_State_t;

typedef enum
{
    AIRBRAKE_FAULT_NONE            = 0x00000000UL,
    AIRBRAKE_FAULT_NOT_CALIBRATED  = 0x00000001UL,
    AIRBRAKE_FAULT_NOT_ALLOWED     = 0x00000002UL,
    AIRBRAKE_FAULT_TMC_STATUS      = 0x00000004UL,
    AIRBRAKE_FAULT_MOVE_TIMEOUT    = 0x00000008UL,
    AIRBRAKE_FAULT_BAD_CONFIG      = 0x00000010UL,
    AIRBRAKE_FAULT_SPI             = 0x00000020UL,
    AIRBRAKE_FAULT_OVERTEMP        = 0x00000040UL,
    AIRBRAKE_FAULT_SHORT           = 0x00000080UL,
    AIRBRAKE_FAULT_UNDERVOLTAGE    = 0x00000100UL
} Airbrake_Fault_t;

typedef struct
{
    int32_t closed_counts;          /* 0% deployment position */
    int32_t open_counts;            /* 100% deployment position */
    int32_t position_tolerance;     /* counts from target considered reached */

    uint32_t vmax_normal;           /* TMC internal velocity units */
    uint32_t amax_normal;           /* TMC internal acceleration units */
    uint32_t dmax_normal;           /* TMC internal deceleration units */

    uint32_t vmax_thermal;          /* reduced speed if thermal warning occurs */
    uint32_t amax_thermal;
    uint32_t dmax_thermal;

    uint8_t ihold_run;              /* moving IHOLD */
    uint8_t irun_run;               /* moving IRUN */
    uint8_t scaler_run;             /* moving GLOBALSCALER */

    uint8_t ihold_hold;             /* holding IHOLD */
    uint8_t irun_hold;              /* holding IRUN */
    uint8_t scaler_hold;            /* holding GLOBALSCALER */

    float launch_accel_g;           /* accel threshold proving launch/thrust */
    float burnout_accel_g;          /* accel below this can indicate burnout */
    float burnout_drop_g;           /* sudden accel drop required for burnout */
    uint32_t post_burnout_delay_ms; /* usually 1000 ms */

    uint32_t move_timeout_ms;
} Airbrake_Config_t;

typedef struct
{
    Airbrake_State_t state;
    uint32_t faults;
    int32_t xactual;
    int32_t target_counts;
    uint8_t current_percent;
    uint8_t target_percent;
    bool calibrated;
    bool launched;
    bool burnout_detected;
    bool deployment_allowed;
    bool manual_override;
    bool thermal_limited;
    uint32_t last_drv_status;
    uint32_t last_gstat;
} Airbrake_Telemetry_t;

void Airbrake_DefaultConfig(Airbrake_Config_t *cfg);
HAL_StatusTypeDef Airbrake_Init(const Airbrake_Config_t *cfg);
HAL_StatusTypeDef Airbrake_CalibrateBounds(int32_t closed_counts, int32_t open_counts, bool zero_current_as_closed);
void Airbrake_SetManualOverride(bool enabled);
void Airbrake_UpdateFlightGate(float accel_g, uint32_t now_ms);
bool Airbrake_IsDeploymentAllowed(void);
HAL_StatusTypeDef Airbrake_SetTargetPercent(uint8_t percent);
HAL_StatusTypeDef Airbrake_Retract(void);
HAL_StatusTypeDef Airbrake_DeployFull(void);
HAL_StatusTypeDef Airbrake_Update(uint32_t now_ms);
void Airbrake_EStop(void);
Airbrake_Telemetry_t Airbrake_GetTelemetry(void);
bool Airbrake_IsBusy(void);

/*
 * Relative motor-step ownership.
 *
 * Positive steps move physically toward deployment.
 * Negative steps move physically toward retraction.
 */
HAL_StatusTypeDef Airbrake_MoveRelativeSteps(int16_t steps);
bool Airbrake_IsManualStepActive(void);

/*
 * Clear recoverable actuator and TMC5240 fault latches without resetting
 * calibration, position, flight state, controller state, or configuration.
 */
HAL_StatusTypeDef Airbrake_ClearFaults(void);

#ifdef __cplusplus
}
#endif

#endif /* AIRBRAKE_H */
