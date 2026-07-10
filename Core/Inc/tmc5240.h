#ifndef TMC5240_H
#define TMC5240_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * Basic TMC5240 driver for STM32 HAL.
 *
 * Assumptions:
 * - SPI2 is configured as full-duplex master.
 * - SPI mode is Mode 3: CPOL = HIGH, CPHA = 2EDGE.
 * - SPI speed is <= 10 MHz.
 * - CS is controlled manually as a GPIO output.
 */

// ==========================
// Register Addresses
// ==========================

#define TMC5240_REG_GCONF          0x00
#define TMC5240_REG_GSTAT          0x01
#define TMC5240_REG_IFCNT          0x02
#define TMC5240_REG_IOIN           0x04

#define TMC5240_REG_DRV_CONF       0x0A
#define TMC5240_REG_GLOBALSCALER   0x0B

#define TMC5240_REG_IHOLD_IRUN     0x10
#define TMC5240_REG_TPOWERDOWN     0x11
#define TMC5240_REG_TSTEP          0x12
#define TMC5240_REG_TPWMTHRS       0x13
#define TMC5240_REG_TCOOLTHRS      0x14
#define TMC5240_REG_THIGH          0x15

#define TMC5240_REG_RAMPMODE       0x20
#define TMC5240_REG_XACTUAL        0x21
#define TMC5240_REG_VACTUAL        0x22
#define TMC5240_REG_VSTART         0x23
#define TMC5240_REG_A1             0x24
#define TMC5240_REG_V1             0x25
#define TMC5240_REG_AMAX           0x26
#define TMC5240_REG_VMAX           0x27
#define TMC5240_REG_DMAX           0x28
#define TMC5240_REG_D1             0x2A
#define TMC5240_REG_VSTOP          0x2B
#define TMC5240_REG_TZEROWAIT      0x2C
#define TMC5240_REG_XTARGET        0x2D
#define TMC5240_REG_V2             0x2E
#define TMC5240_REG_A2             0x2F
#define TMC5240_REG_D2             0x30
#define TMC5240_REG_RAMP_STAT      0x35

#define TMC5240_REG_CHOPCONF       0x6C
#define TMC5240_REG_DRV_STATUS     0x6F
#define TMC5240_REG_PWMCONF        0x70

// ==========================
// RAMPMODE Values
// ==========================

#define TMC5240_RAMPMODE_POSITION      0x00
#define TMC5240_RAMPMODE_VEL_POSITIVE  0x01
#define TMC5240_RAMPMODE_VEL_NEGATIVE  0x02
#define TMC5240_RAMPMODE_HOLD          0x03

// ==========================
// Safety Limits
// ==========================

#define TMC5240_SAFE_MAX_IHOLD          20U
#define TMC5240_SAFE_MAX_IRUN           31U
#define TMC5240_SAFE_MAX_GLOBALSCALER   128U

// DRV_STATUS safety bits
#define TMC5240_DRV_STST        (1UL << 31)
#define TMC5240_DRV_OLB         (1UL << 30)
#define TMC5240_DRV_OLA         (1UL << 29)
#define TMC5240_DRV_S2GB        (1UL << 28)
#define TMC5240_DRV_S2GA        (1UL << 27)
#define TMC5240_DRV_OTPW        (1UL << 26)
#define TMC5240_DRV_OT          (1UL << 25)
#define TMC5240_DRV_STALLGUARD  (1UL << 24)
#define TMC5240_DRV_S2VSB       (1UL << 13)
#define TMC5240_DRV_S2VSA       (1UL << 12)

#define TMC5240_DRV_CS_ACTUAL_MASK   (0x1FUL << 16)
#define TMC5240_DRV_CS_ACTUAL_SHIFT  16

// GSTAT bits
#define TMC5240_GSTAT_RESET          (1UL << 0)
#define TMC5240_GSTAT_DRV_ERR        (1UL << 1)
#define TMC5240_GSTAT_UV_CP          (1UL << 2)
#define TMC5240_GSTAT_REGISTER_RESET (1UL << 3)
#define TMC5240_GSTAT_VM_UVLO        (1UL << 4)

// ==========================
// Basic Driver API
// ==========================

HAL_StatusTypeDef TMC5240_WriteReg(uint8_t reg, uint32_t value);
HAL_StatusTypeDef TMC5240_ReadReg(uint8_t reg, uint32_t *value);

void TMC5240_Wake(void);
void TMC5240_Sleep(void);
HAL_StatusTypeDef TMC5240_EnableDriver(void);
void TMC5240_DisableDriver(void);

HAL_StatusTypeDef TMC5240_InitBasic(void);

HAL_StatusTypeDef TMC5240_MoveVelocity(uint32_t vmax, uint32_t amax,bool positive);
HAL_StatusTypeDef TMC5240_StopVelocity(void);

HAL_StatusTypeDef TMC5240_MoveToPosition(int32_t target_position);
HAL_StatusTypeDef TMC5240_SetCurrent(uint8_t ihold, uint8_t irun);

HAL_StatusTypeDef TMC5240_GetIOIN(uint32_t *ioin);
HAL_StatusTypeDef TMC5240_GetGSTAT(uint32_t *gstat);
HAL_StatusTypeDef TMC5240_GetDRV_STATUS(uint32_t *drv_status);
HAL_StatusTypeDef TMC5240_GetXACTUAL(int32_t *xactual);

HAL_StatusTypeDef TMC5240_CheckSafety(uint32_t *drv_status, uint32_t *gstat);
HAL_StatusTypeDef TMC5240_SetGlobalScaler(uint8_t scaler);

uint32_t TMC5240_GetLastDRVStatus(void);
uint32_t TMC5240_GetLastGSTAT(void);

#endif
