#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

// 7-bit I2C addresses. Use the unshifted value in the handle.
// SA0 = 0 -> 0x6A. SA0 = 1 -> 0x6B.
#define LSM6DSV32X_I2C_ADDR_LOW      0x6AU
#define LSM6DSV32X_I2C_ADDR_HIGH     0x6BU

#define LSM6DSV32X_WHO_AM_I_REG      0x0FU
#define LSM6DSV32X_WHO_AM_I_VALUE    0x70U

#define LSM6DSV32X_STATUS_REG        0x1EU
#define LSM6DSV32X_OUT_TEMP_L        0x20U
#define LSM6DSV32X_OUTX_L_G          0x22U
#define LSM6DSV32X_OUTX_L_A          0x28U

#define LSM6DSV32X_CTRL1             0x10U
#define LSM6DSV32X_CTRL2             0x11U
#define LSM6DSV32X_CTRL3             0x12U
#define LSM6DSV32X_CTRL6             0x15U
#define LSM6DSV32X_CTRL8             0x17U

// Data order returned by LSM6DSV32X_ReadData()
typedef enum
{
    LSM6DSV32X_GYRO_X = 0,
    LSM6DSV32X_GYRO_Y = 1,
    LSM6DSV32X_GYRO_Z = 2,
    LSM6DSV32X_ACCEL_X = 3,
    LSM6DSV32X_ACCEL_Y = 4,
    LSM6DSV32X_ACCEL_Z = 5,
    LSM6DSV32X_DATA_COUNT = 6
} LSM6DSV32X_DataIndex_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint8_t who_am_i;
} LSM6DSV32X_HandleTypeDef;

HAL_StatusTypeDef LSM6DSV32X_Init(LSM6DSV32X_HandleTypeDef *dev);
HAL_StatusTypeDef LSM6DSV32X_ReadWhoAmI(LSM6DSV32X_HandleTypeDef *dev, uint8_t *who_am_i);
HAL_StatusTypeDef LSM6DSV32X_ReadStatus(LSM6DSV32X_HandleTypeDef *dev, uint8_t *status);

// Fills data[6] as: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z.
// Raw two's-complement values. Convert using the full-scale settings you selected.
HAL_StatusTypeDef LSM6DSV32X_ReadData(LSM6DSV32X_HandleTypeDef *dev, float data[LSM6DSV32X_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSV32X_H
