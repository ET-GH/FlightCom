#include "tmc5240.h"
extern SPI_HandleTypeDef hspi2;

#define TMC5240_SPI_TIMEOUT_MS  100

#define TMC5240_DRV_HARD_FAULT_MASK \
    (TMC5240_DRV_S2GB | TMC5240_DRV_S2GA | \
     TMC5240_DRV_S2VSB | TMC5240_DRV_S2VSA | \
     TMC5240_DRV_OTPW | TMC5240_DRV_OT)

#define TMC5240_GSTAT_FAULT_MASK \
    (TMC5240_GSTAT_DRV_ERR | TMC5240_GSTAT_UV_CP | \
     TMC5240_GSTAT_REGISTER_RESET | TMC5240_GSTAT_VM_UVLO)

static volatile uint32_t tmc5240_last_drv_status = 0;
static volatile uint32_t tmc5240_last_gstat = 0;

static void TMC5240_AbortMotionUnsafe(void);

// ==========================
// Internal Helpers
// ==========================

static void TMC5240_CS_Low(void)
{
    HAL_GPIO_WritePin(MOTOR_CS_GPIO_Port, MOTOR_CS_Pin, GPIO_PIN_RESET);
}

static void TMC5240_CS_High(void)
{
    HAL_GPIO_WritePin(MOTOR_CS_GPIO_Port, MOTOR_CS_Pin, GPIO_PIN_SET);
}

static uint32_t TMC5240_U32_FromBytes(const uint8_t rx[5])
{
    return ((uint32_t)rx[1] << 24) |
           ((uint32_t)rx[2] << 16) |
           ((uint32_t)rx[3] << 8)  |
           ((uint32_t)rx[4]);
}

static HAL_StatusTypeDef TMC5240_Transfer5(uint8_t tx[5], uint8_t rx[5])
{
    HAL_StatusTypeDef status;

    TMC5240_CS_Low();

    status = HAL_SPI_TransmitReceive(
        &hspi2,
        tx,
        rx,
        5,
        TMC5240_SPI_TIMEOUT_MS
    );

    TMC5240_CS_High();

    return status;
}

static uint32_t TMC5240_Make_IHOLD_IRUN(uint8_t ihold,
                                        uint8_t irun,
                                        uint8_t iholddelay,
                                        uint8_t irundelay)
{
    ihold      &= 0x1F;
    irun       &= 0x1F;
    iholddelay &= 0x0F;
    irundelay  &= 0x0F;

    return ((uint32_t)irundelay  << 24) |
           ((uint32_t)iholddelay << 16) |
           ((uint32_t)irun       << 8)  |
           ((uint32_t)ihold);
}

// ==========================
// Raw Register Access
// ==========================

HAL_StatusTypeDef TMC5240_WriteReg(uint8_t reg, uint32_t value)
{
    uint8_t tx[5];
    uint8_t rx[5];

    tx[0] = reg | 0x80;        // MSB = 1 means write
    tx[1] = (uint8_t)(value >> 24);
    tx[2] = (uint8_t)(value >> 16);
    tx[3] = (uint8_t)(value >> 8);
    tx[4] = (uint8_t)(value);

    return TMC5240_Transfer5(tx, rx);
}

HAL_StatusTypeDef TMC5240_ReadReg(uint8_t reg, uint32_t *value)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    tx[0] = reg & 0x7F;        // MSB = 0 means read
    tx[1] = 0x00;
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;

    /*
     * TMC5240 reads are pipelined:
     * first transaction requests the register,
     * second transaction receives the requested data.
     */
    status = TMC5240_Transfer5(tx, rx);
    if (status != HAL_OK)
    {
        return status;
    }

    status = TMC5240_Transfer5(tx, rx);
    if (status != HAL_OK)
    {
        return status;
    }

    *value = TMC5240_U32_FromBytes(rx);
    return HAL_OK;
}

// ==========================
// Hardware Control Pins
// ==========================

void TMC5240_Wake(void)
{
    // CS idle high.
    TMC5240_CS_High();

    // Keep bridge disabled during configuration.
    TMC5240_DisableDriver();

    // SLEEPN is active low. High wakes the chip.
    HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_SET);

    // Datasheet wake-up time is typically 2.5 ms.
    HAL_Delay(3);
}

void TMC5240_Sleep(void)
{
    TMC5240_DisableDriver();

    // SLEEPN low puts the chip into sleep/reset.
    HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_RESET);

    HAL_Delay(1);
}

HAL_StatusTypeDef TMC5240_EnableDriver(void)
{
    if (TMC5240_CheckSafety(NULL, NULL) != HAL_OK)
    {
        TMC5240_DisableDriver();
        return HAL_ERROR;
    }

    // DRV_ENN is active low: low enables the output bridge.
    HAL_GPIO_WritePin(MOTOR_DRV_ENN_GPIO_Port, MOTOR_DRV_ENN_Pin, GPIO_PIN_RESET);

    HAL_Delay(1);

    return TMC5240_CheckSafety(NULL, NULL);
}

void TMC5240_DisableDriver(void)
{
    // DRV_ENN high disables the output bridge.
    HAL_GPIO_WritePin(MOTOR_DRV_ENN_GPIO_Port, MOTOR_DRV_ENN_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef TMC5240_SetGlobalScaler(uint8_t scaler)
{
    /* Valid hardware values are 0 or 32 through 255.
       Zero is the special full-scale setting. */
    if ((scaler != 0U) && (scaler < 32U))
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    return TMC5240_WriteReg(
        TMC5240_REG_GLOBALSCALER,
        (uint32_t)scaler
    );
}

// ==========================
// Basic Configuration
// ==========================

HAL_StatusTypeDef TMC5240_InitBasic(void)
{
    HAL_StatusTypeDef status;
    uint32_t dummy;

    TMC5240_Wake();

    /*
     * Optional communication test.
     * IOIN contains a VERSION field in the upper byte.
     */
    status = TMC5240_ReadReg(TMC5240_REG_IOIN, &dummy);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Clear GSTAT flags.
     * Writing 1 clears write-clear status bits.
     */
    status = TMC5240_WriteReg(TMC5240_REG_GSTAT, 0x0000001F);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Use SpreadCycle first because it is simpler for bring-up.
     * GCONF en_pwm_mode = 0 means not using StealthChop mode.
     */
    status = TMC5240_WriteReg(TMC5240_REG_GCONF, 0x00000000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_DRV_CONF, 0x00000000);
	if (status != HAL_OK)
	{
		TMC5240_DisableDriver();
		return status;
	}

    /*
     * Low conservative current for first test.
     *
     * IHOLD = 4
     * IRUN  = 8
     *
     * Tune these later based on your motor and IREF hardware.
     */
    status = TMC5240_SetCurrent(4, 8);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Global current scaler.
     * 32 is conservative. Increase later only after confirming motor current.
     */
    status = TMC5240_SetGlobalScaler(32);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Standstill current reduction delay.
     */
    status = TMC5240_WriteReg(TMC5240_REG_TPOWERDOWN, 10);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Basic SpreadCycle CHOPCONF.
     *
     * intpol = 1
     * TBL    = 2
     * HSTRT  = 0
     * HEND   = 0
     * TOFF   = 5
     *
     * TOFF > 0 enables the chopper.
     */
    status = TMC5240_WriteReg(TMC5240_REG_CHOPCONF, 0x10010005);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    /*
     * Basic ramp values.
     */
    status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE, TMC5240_RAMPMODE_HOLD);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_XACTUAL, 0);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VSTART, 0);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_A1, 1000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_V1, 50000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_AMAX, 1000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, 0);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_DMAX, 1000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_D1, 1000);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VSTOP, 100);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_TZEROWAIT, 10);
    if (status != HAL_OK)
    {
        TMC5240_DisableDriver();
        return status;
    }

    return TMC5240_EnableDriver();
}

HAL_StatusTypeDef TMC5240_SetCurrent(uint8_t ihold, uint8_t irun)
{
    uint32_t value;

    if (ihold > TMC5240_SAFE_MAX_IHOLD)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    if (irun > TMC5240_SAFE_MAX_IRUN)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    if (ihold > irun)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    value = TMC5240_Make_IHOLD_IRUN(
        ihold,
        irun,
        4,      // IHOLDDELAY
        1       // IRUNDELAY
    );

    return TMC5240_WriteReg(TMC5240_REG_IHOLD_IRUN, value);
}

static void TMC5240_AbortMotionUnsafe(void)
{
    /*
     * This intentionally uses raw register writes.
     * Do not call the safety checker from inside this function,
     * or fault handling can become recursive.
     */
    (void)TMC5240_WriteReg(TMC5240_REG_VMAX, 0);
    (void)TMC5240_WriteReg(TMC5240_REG_RAMPMODE, TMC5240_RAMPMODE_HOLD);

    // DRV_ENN high disables the output bridge.
    TMC5240_DisableDriver();
}

HAL_StatusTypeDef TMC5240_CheckSafety(uint32_t *drv_status, uint32_t *gstat)
{
    uint32_t drv = 0;
    uint32_t gs = 0;

    if (TMC5240_ReadReg(TMC5240_REG_DRV_STATUS, &drv) != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    if (TMC5240_ReadReg(TMC5240_REG_GSTAT, &gs) != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    tmc5240_last_drv_status = drv;
    tmc5240_last_gstat = gs;

    if (drv_status != NULL)
    {
        *drv_status = drv;
    }

    if (gstat != NULL)
    {
        *gstat = gs;
    }

    /*
     * Abort on short-to-ground, short-to-supply,
     * overtemperature prewarning, or overtemperature shutdown.
     */
    if ((drv & TMC5240_DRV_HARD_FAULT_MASK) != 0U)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    /*
     * Abort on charge pump, driver error, register reset,
     * or motor-supply undervoltage.
     *
     * Do not include GSTAT_RESET here if you clear GSTAT during init.
     */
    if ((gs & TMC5240_GSTAT_FAULT_MASK) != 0U)
    {
        TMC5240_AbortMotionUnsafe();
        return HAL_ERROR;
    }

    return HAL_OK;
}

uint32_t TMC5240_GetLastDRVStatus(void)
{
    return tmc5240_last_drv_status;
}

uint32_t TMC5240_GetLastGSTAT(void)
{
    return tmc5240_last_gstat;
}

// ==========================
// Motion Commands
// ==========================

HAL_StatusTypeDef TMC5240_MoveVelocity(uint32_t vmax, uint32_t amax, bool positive)
{
    HAL_StatusTypeDef status;

    if (TMC5240_CheckSafety(NULL, NULL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    status = TMC5240_WriteReg(TMC5240_REG_AMAX, amax);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, vmax);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    if (positive)
    {
        status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE,
                                  TMC5240_RAMPMODE_VEL_POSITIVE);
    }
    else
    {
        status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE,
                                  TMC5240_RAMPMODE_VEL_NEGATIVE);
    }

    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    return TMC5240_CheckSafety(NULL, NULL);
}

HAL_StatusTypeDef TMC5240_StopVelocity(void)
{
    HAL_StatusTypeDef status;

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, 0);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE, TMC5240_RAMPMODE_HOLD);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_MoveToPosition(int32_t target_position)
{
    HAL_StatusTypeDef status;

    if (TMC5240_CheckSafety(NULL, NULL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    status = TMC5240_WriteReg(TMC5240_REG_RAMPMODE,
                              TMC5240_RAMPMODE_POSITION);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_VMAX, 20000);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_AMAX, 50);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_DMAX, 50);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    status = TMC5240_WriteReg(TMC5240_REG_XTARGET,
                              (uint32_t)target_position);
    if (status != HAL_OK)
    {
        TMC5240_AbortMotionUnsafe();
        return status;
    }

    return TMC5240_CheckSafety(NULL, NULL);
}

// ==========================
// Status Helpers
// ==========================

HAL_StatusTypeDef TMC5240_GetIOIN(uint32_t *ioin)
{
    return TMC5240_ReadReg(TMC5240_REG_IOIN, ioin);
}

HAL_StatusTypeDef TMC5240_GetGSTAT(uint32_t *gstat)
{
    return TMC5240_ReadReg(TMC5240_REG_GSTAT, gstat);
}

HAL_StatusTypeDef TMC5240_GetDRV_STATUS(uint32_t *drv_status)
{
    return TMC5240_ReadReg(TMC5240_REG_DRV_STATUS, drv_status);
}

HAL_StatusTypeDef TMC5240_GetXACTUAL(int32_t *xactual)
{
    uint32_t raw;
    HAL_StatusTypeDef status;

    if (xactual == NULL)
    {
        return HAL_ERROR;
    }

    status = TMC5240_ReadReg(TMC5240_REG_XACTUAL, &raw);
    if (status != HAL_OK)
    {
        return status;
    }

    *xactual = (int32_t)raw;
    return HAL_OK;
}
