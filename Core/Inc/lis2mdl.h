#ifndef LIS2MDL_H
#define LIS2MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

#define LIS2MDL_I2C_ADDR             0x1EU
#define LIS2MDL_WHO_AM_I_REG         0x4FU
#define LIS2MDL_WHO_AM_I_VALUE       0x40U

#define LIS2MDL_CFG_REG_A            0x60U
#define LIS2MDL_CFG_REG_B            0x61U
#define LIS2MDL_CFG_REG_C            0x62U
#define LIS2MDL_STATUS_REG           0x67U
#define LIS2MDL_OUTX_L_REG           0x68U

// Data order returned by LIS2MDL_ReadData()
typedef enum
{
    LIS2MDL_MAG_X = 0,
    LIS2MDL_MAG_Y = 1,
    LIS2MDL_MAG_Z = 2,
    LIS2MDL_DATA_COUNT = 3
} LIS2MDL_DataIndex_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint8_t who_am_i;
} LIS2MDL_HandleTypeDef;

HAL_StatusTypeDef LIS2MDL_Init(LIS2MDL_HandleTypeDef *dev);
HAL_StatusTypeDef LIS2MDL_ReadWhoAmI(LIS2MDL_HandleTypeDef *dev, uint8_t *who_am_i);
HAL_StatusTypeDef LIS2MDL_ReadStatus(LIS2MDL_HandleTypeDef *dev, uint8_t *status);

// Fills data[3] as: mag_x, mag_y, mag_z. Raw two's-complement magnetic readings.
HAL_StatusTypeDef LIS2MDL_ReadData(LIS2MDL_HandleTypeDef *dev, float data[LIS2MDL_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // LIS2MDL_H
