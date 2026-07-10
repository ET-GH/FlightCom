#ifndef ROCKET_SENSORS_H
#define ROCKET_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lsm6dsv32x.h"
#include "lis2mdl.h"
#include "bmp388.h"

extern LSM6DSV32X_HandleTypeDef rocket_imu;
extern LIS2MDL_HandleTypeDef rocket_mag;
extern BMP388_HandleTypeDef rocket_baro;

typedef struct
{
	float imu[LSM6DSV32X_DATA_COUNT];     // gyro_x/y/z, accel_x/y/z
	float mag[LIS2MDL_DATA_COUNT];        // mag_x/y/z
    float baro[BMP388_DATA_COUNT];       // raw_pressure, raw_temperature
} RocketSensorRawData_t;

HAL_StatusTypeDef RocketSensors_Init(void);
HAL_StatusTypeDef RocketSensors_ReadAll(
    RocketSensorRawData_t *data,
    HAL_StatusTypeDef *imu_status,
    HAL_StatusTypeDef *mag_status,
    HAL_StatusTypeDef *baro_status
);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_SENSORS_H
