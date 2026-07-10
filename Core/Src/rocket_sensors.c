#include "rocket_sensors.h"
//#include "i2c.h"
//#include "main.c"
//
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
// Edit these if your schematic assigns the sensors to different I2C buses.
#ifndef ROCKET_IMU_I2C_HANDLE
#define ROCKET_IMU_I2C_HANDLE   hi2c1
#endif

#ifndef ROCKET_BARO_I2C_HANDLE
#define ROCKET_BARO_I2C_HANDLE  hi2c2
#endif

#ifndef ROCKET_MAG_I2C_HANDLE
#define ROCKET_MAG_I2C_HANDLE   hi2c3
#endif

// Edit address choices if the address-select pin is wired differently.
LSM6DSV32X_HandleTypeDef rocket_imu = {
    .hi2c = &ROCKET_IMU_I2C_HANDLE,
    .addr_7bit = LSM6DSV32X_I2C_ADDR_LOW,
    .who_am_i = 0U
};

LIS2MDL_HandleTypeDef rocket_mag = {
    .hi2c = &ROCKET_MAG_I2C_HANDLE,
    .addr_7bit = LIS2MDL_I2C_ADDR,
    .who_am_i = 0U
};

BMP388_HandleTypeDef rocket_baro = {
    .hi2c = &ROCKET_BARO_I2C_HANDLE,
    .addr_7bit = BMP388_I2C_ADDR_LOW,
    .chip_id = 0U
};

HAL_StatusTypeDef RocketSensors_Init(void)
{
    HAL_StatusTypeDef status;

    status = LSM6DSV32X_Init(&rocket_imu);
    if (status != HAL_OK) { return status; }

    status = LIS2MDL_Init(&rocket_mag);
    if (status != HAL_OK) { return status; }

    status = BMP388_Init(&rocket_baro);
    if (status != HAL_OK) { return status; }

    return HAL_OK;
}

HAL_StatusTypeDef RocketSensors_ReadAll(
    RocketSensorRawData_t *data,
    HAL_StatusTypeDef *imu_status,
    HAL_StatusTypeDef *mag_status,
    HAL_StatusTypeDef *baro_status
)
{
    HAL_StatusTypeDef overall_status = HAL_OK;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (imu_status == NULL || mag_status == NULL || baro_status == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Try every sensor independently.
     * Do not return early if one fails.
     */

    *imu_status = LSM6DSV32X_ReadData(&rocket_imu, data->imu);
    if (*imu_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    *mag_status = LIS2MDL_ReadData(&rocket_mag, data->mag);
    if (*mag_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    *baro_status = BMP388_ReadData(&rocket_baro, data->baro);
    if (*baro_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    return overall_status;
}
