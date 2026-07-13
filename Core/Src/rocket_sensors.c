/**
 * @file rocket_sensors.c
 * @brief Board-level ownership and coordinated access for the flight sensors.
 *
 * This module binds each sensor driver to its configured STM32 I2C peripheral,
 * exposes the shared sensor handles used by the behavior layer, and provides
 * convenience functions for initialization and full raw-data acquisition.
 *
 * Bus-selection macros may be overridden by the build or a board-specific
 * header without modifying this source file.
 */

#include "rocket_sensors.h"

#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* STM32 I2C peripheral bindings                                              */
/* -------------------------------------------------------------------------- */

/* These handles are generated and initialized by STM32CubeMX/CubeIDE. */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/** Default I2C peripheral used by the LSM6DSV32X IMU. */
#ifndef ROCKET_IMU_I2C_HANDLE
#define ROCKET_IMU_I2C_HANDLE   hi2c1
#endif

/** Default I2C peripheral used by the BMP388 barometer. */
#ifndef ROCKET_BARO_I2C_HANDLE
#define ROCKET_BARO_I2C_HANDLE  hi2c2
#endif

/** Default I2C peripheral used by the LIS2MDL magnetometer. */
#ifndef ROCKET_MAG_I2C_HANDLE
#define ROCKET_MAG_I2C_HANDLE   hi2c3
#endif

/* -------------------------------------------------------------------------- */
/* Shared sensor handles                                                      */
/* -------------------------------------------------------------------------- */

/**
 * LSM6DSV32X IMU instance.
 *
 * Change the address macro when the IMU address-select pin is wired high.
 */
LSM6DSV32X_HandleTypeDef rocket_imu = {
    .hi2c = &ROCKET_IMU_I2C_HANDLE,
    .addr_7bit = LSM6DSV32X_I2C_ADDR_LOW,
    .who_am_i = 0U
};

/** LIS2MDL magnetometer instance. */
LIS2MDL_HandleTypeDef rocket_mag = {
    .hi2c = &ROCKET_MAG_I2C_HANDLE,
    .addr_7bit = LIS2MDL_I2C_ADDR,
    .who_am_i = 0U
};

/**
 * BMP388 barometer instance.
 *
 * Change the address macro when the barometer address-select pin is wired high.
 */
BMP388_HandleTypeDef rocket_baro = {
    .hi2c = &ROCKET_BARO_I2C_HANDLE,
    .addr_7bit = BMP388_I2C_ADDR_LOW,
    .chip_id = 0U
};

/* ========================================================================== */
/* Public sensor API                                                          */
/* ========================================================================== */

/**
 * @brief Initialize all flight sensors in dependency-independent order.
 *
 * Initialization stops at the first failure and returns that driver's HAL
 * status. The failing driver can therefore be identified from the call order:
 * IMU, magnetometer, then barometer.
 *
 * @return HAL_OK when every sensor initializes successfully; otherwise the
 *         status returned by the first failed driver.
 */
HAL_StatusTypeDef RocketSensors_Init(void)
{
    HAL_StatusTypeDef status;

    status = LSM6DSV32X_Init(&rocket_imu);
    if (status != HAL_OK)
    {
        return status;
    }

    status = LIS2MDL_Init(&rocket_mag);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP388_Init(&rocket_baro);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Read raw data from the IMU, magnetometer, and barometer.
 *
 * Every sensor read is attempted even when an earlier read fails. This allows
 * callers to inspect the three individual status outputs and determine which
 * devices remain operational. The aggregate return value is HAL_ERROR when any
 * individual read is not HAL_OK.
 *
 * @param data Destination for all raw sensor arrays.
 * @param imu_status Receives the LSM6DSV32X read status.
 * @param mag_status Receives the LIS2MDL read status.
 * @param baro_status Receives the BMP388 read status.
 * @return HAL_OK only when all three reads succeed; otherwise HAL_ERROR.
 */
HAL_StatusTypeDef RocketSensors_ReadAll(
    RocketSensorRawData_t *data,
    HAL_StatusTypeDef *imu_status,
    HAL_StatusTypeDef *mag_status,
    HAL_StatusTypeDef *baro_status)
{
    HAL_StatusTypeDef overall_status = HAL_OK;

    if ((data == NULL) ||
        (imu_status == NULL) ||
        (mag_status == NULL) ||
        (baro_status == NULL))
    {
        return HAL_ERROR;
    }

    /* Do not return early after an individual read failure. The per-device
     * statuses are part of the function's output and should all be updated. */
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
