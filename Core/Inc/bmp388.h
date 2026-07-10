#ifndef BMP388_H
#define BMP388_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* Standard sea-level pressure used for approximate altitude above sea level. */
#define BMP388_STANDARD_SEA_LEVEL_PRESSURE_PA 101325.0f

/* 7-bit I2C addresses. Store the unshifted address in the device handle. */
#define BMP388_I2C_ADDR_LOW              0x76U  /* SDO = GND */
#define BMP388_I2C_ADDR_HIGH             0x77U  /* SDO = VDDIO */

#define BMP388_CHIP_ID_REG               0x00U
#define BMP388_CHIP_ID_VALUE             0x50U
#define BMP388_ERR_REG                   0x02U
#define BMP388_STATUS_REG                0x03U
#define BMP388_DATA_0                    0x04U
#define BMP388_CALIB_DATA_START          0x31U
#define BMP388_CALIB_DATA_LENGTH         21U
#define BMP388_PWR_CTRL                  0x1BU
#define BMP388_OSR                       0x1CU
#define BMP388_ODR                       0x1DU
#define BMP388_CONFIG                    0x1FU
#define BMP388_CMD                       0x7EU

#define BMP388_CMD_SOFT_RESET            0xB6U

/*
 * The same array order is used by the raw and compensated read functions.
 * Compensated pressure is returned in Pa and temperature in degrees Celsius.
 */
typedef enum
{
    BMP388_PRESSURE = 0,
    BMP388_TEMPERATURE = 1,
    BMP388_DATA_COUNT = 2
} BMP388_DataIndex_t;

/* Backward-compatible names for code using the previous raw-data enum. */
#define BMP388_RAW_PRESSURE              BMP388_PRESSURE
#define BMP388_RAW_TEMPERATURE           BMP388_TEMPERATURE
#define BMP388_PRESSURE_PA               BMP388_PRESSURE
#define BMP388_TEMPERATURE_C             BMP388_TEMPERATURE

/* Floating-point calibration coefficients after conversion from sensor NVM. */
typedef struct BMP388_calib_data
{
    float par_t1;
    float par_t2;
    float par_t3;

    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;

    /* Linearized temperature used by the pressure compensation equation. */
    float t_lin;
} BMP388_calib_data;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint8_t chip_id;
    uint8_t calib_valid;
    uint8_t reference_pressure_valid;
    float reference_pressure_pa;
    BMP388_calib_data calib;
} BMP388_HandleTypeDef;

HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev);
HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id);
HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status);
HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error);

/*
 * Reads uncompensated 24-bit ADC values.
 * raw_data[BMP388_PRESSURE]    = raw pressure
 * raw_data[BMP388_TEMPERATURE] = raw temperature
 */
HAL_StatusTypeDef BMP388_ReadRawData(
    BMP388_HandleTypeDef *dev,
    uint32_t raw_data[BMP388_DATA_COUNT]);

/*
 * Reads and compensates one pressure/temperature sample.
 * data[BMP388_PRESSURE]    = pressure in Pa
 * data[BMP388_TEMPERATURE] = temperature in degrees Celsius
 */
HAL_StatusTypeDef BMP388_ReadData(
    BMP388_HandleTypeDef *dev,
    float data[BMP388_DATA_COUNT]);

/*
 * Converts compensated pressure to pressure altitude in meters.
 * pressure_pa must be in Pa. reference_pressure_pa is normally either:
 *   - 101325 Pa for approximate altitude above mean sea level, or
 *   - the measured launch-pad pressure for altitude relative to the pad.
 * Returns NAN if either pressure is invalid.
 */
float BMP388_PressureToAltitude(
    float pressure_pa,
    float reference_pressure_pa);

/* Stores the pressure that corresponds to altitude = 0 m. */
HAL_StatusTypeDef BMP388_SetReferencePressure(
    BMP388_HandleTypeDef *dev,
    float reference_pressure_pa);

/*
 * Averages multiple compensated pressure samples and stores the result as the
 * zero-altitude reference. Keep the sensor stationary during calibration.
 */
HAL_StatusTypeDef BMP388_CalibrateAltitudeReference(
    BMP388_HandleTypeDef *dev,
    uint16_t sample_count,
    uint32_t sample_delay_ms);

/*
 * Reads the sensor and returns altitude in meters relative to the currently
 * stored reference pressure.
 */
HAL_StatusTypeDef BMP388_ReadAltitude(
    BMP388_HandleTypeDef *dev,
    float *altitude_m);

#ifdef __cplusplus
}
#endif

#endif /* BMP388_H */
