/*
 * BMP388 pressure-sensor driver
 *
 * This file contains the STM32 HAL/I2C implementation for:
 *   1. Detecting and resetting the BMP388.
 *   2. Reading the factory calibration coefficients.
 *   3. Configuring continuous pressure/temperature measurements.
 *   4. Waiting for a fresh measurement before returning data.
 *   5. Converting raw ADC values into pressure, temperature, and altitude.
 *
 * The public function declarations and the BMP388_HandleTypeDef definition are
 * expected to be provided by bmp388.h.
 */

#include "bmp388.h"

#include <math.h>
#include <stddef.h>

/*
 * A normal I2C transaction should complete much faster than this. A relatively
 * short timeout prevents a disconnected sensor from blocking the flight loop
 * for a long period.
 */
#define BMP388_I2C_TIMEOUT_MS                 10U

/* Time allowed for reset/configuration status changes. */
#define BMP388_RESET_TIMEOUT_MS               25U
#define BMP388_DATA_READY_TIMEOUT_MS          30U

/* STATUS register bit masks (datasheet register 0x03). */
#define BMP388_STATUS_CMD_READY_MASK          (1U << 4)
#define BMP388_STATUS_PRESS_READY_MASK        (1U << 5)
#define BMP388_STATUS_TEMP_READY_MASK         (1U << 6)
#define BMP388_STATUS_DATA_READY_MASK         \
    (BMP388_STATUS_PRESS_READY_MASK | BMP388_STATUS_TEMP_READY_MASK)

/* ERR_REG bits that indicate an actual sensor/configuration failure. */
#define BMP388_ERROR_MASK                     0x07U

/*
 * Flight configuration used by BMP388_Init().
 *
 * OSR = 0x02:
 *   pressure oversampling  = x4
 *   temperature oversampling = x1
 *
 * ODR = 0x02:
 *   50 Hz output data rate, or one new sample every 20 ms.
 *
 * CONFIG = 0x04:
 *   IIR coefficient 3. This reduces pressure noise while keeping much less
 *   delay than the larger IIR coefficients. Use 0x00 to bypass the IIR filter.
 *
 * PWR_CTRL = 0x33:
 *   pressure enabled, temperature enabled, normal/continuous mode.
 *
 * Pressure x4 and temperature x1 require at most about 12.5 ms per conversion,
 * so the 20 ms period at 50 Hz leaves sufficient conversion time.
 */
#define BMP388_CONFIG_OSR_VALUE               0x02U
#define BMP388_CONFIG_ODR_VALUE               0x02U
#define BMP388_CONFIG_IIR_VALUE               0x04U
#define BMP388_CONFIG_PWR_CTRL_VALUE          0x33U

/* Masks used when reading configuration registers back for verification. */
#define BMP388_OSR_VERIFY_MASK                0x3FU
#define BMP388_ODR_VERIFY_MASK                0x1FU
#define BMP388_CONFIG_VERIFY_MASK             0x0EU
#define BMP388_PWR_CTRL_VERIFY_MASK           0x33U

/*
 * STM32 HAL expects the 7-bit I2C address shifted left by one bit. The handle
 * must therefore contain 0x76 or 0x77, not the already shifted 0xEC/0xEE value.
 */
#define BMP388_ADDR(dev) \
    ((uint16_t)((uint16_t)(dev)->addr_7bit << 1U))

/* Write one byte to a BMP388 register. */
static HAL_StatusTypeDef bmp388_write_u8(
    BMP388_HandleTypeDef *dev,
    uint8_t reg,
    uint8_t value)
{
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             BMP388_ADDR(dev),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             BMP388_I2C_TIMEOUT_MS);
}

/* Read one byte from a BMP388 register. */
static HAL_StatusTypeDef bmp388_read_u8(
    BMP388_HandleTypeDef *dev,
    uint8_t reg,
    uint8_t *value)
{
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            BMP388_ADDR(dev),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            BMP388_I2C_TIMEOUT_MS);
}

/*
 * Read several consecutive registers in one I2C transaction. Burst reads are
 * important for the six data bytes because they guarantee that pressure and
 * temperature belong to the same measurement cycle.
 */
static HAL_StatusTypeDef bmp388_read_bytes(
    BMP388_HandleTypeDef *dev,
    uint8_t start_reg,
    uint8_t *buffer,
    uint16_t len)
{
    if (dev == NULL || dev->hi2c == NULL || buffer == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            BMP388_ADDR(dev),
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            BMP388_I2C_TIMEOUT_MS);
}

/* Convert two little-endian bytes into an unsigned 16-bit value. */
static uint16_t bmp388_u16_le(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[1] << 8U) | (uint16_t)bytes[0]);
}

/*
 * Poll STATUS until every bit in mask has the requested value or the timeout
 * expires. HAL_GetTick() subtraction remains valid when the tick counter wraps.
 */
static HAL_StatusTypeDef bmp388_wait_for_status(
    BMP388_HandleTypeDef *dev,
    uint8_t mask,
    uint8_t expected,
    uint32_t timeout_ms)
{
    const uint32_t start_tick = HAL_GetTick();
    uint8_t sensor_status = 0U;
    HAL_StatusTypeDef status;

    do
    {
        status = BMP388_ReadStatus(dev, &sensor_status);
        if (status != HAL_OK)
        {
            return status;
        }

        if ((sensor_status & mask) == expected)
        {
            return HAL_OK;
        }

        HAL_Delay(1U);
    }
    while ((HAL_GetTick() - start_tick) < timeout_ms);

    return HAL_TIMEOUT;
}

/*
 * Verify only the documented writable bits of a configuration register. This
 * catches failed writes without being affected by reserved register bits.
 */
static HAL_StatusTypeDef bmp388_verify_register(
    BMP388_HandleTypeDef *dev,
    uint8_t reg,
    uint8_t expected,
    uint8_t mask)
{
    uint8_t actual = 0U;
    HAL_StatusTypeDef status = bmp388_read_u8(dev, reg, &actual);

    if (status != HAL_OK)
    {
        return status;
    }

    if ((actual & mask) != (expected & mask))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*
 * Read ERR_REG and convert any BMP388-reported error into HAL_ERROR.
 * cmd_err and conf_err are clear-on-read, so this function also acknowledges
 * those flags.
 */
static HAL_StatusTypeDef bmp388_check_sensor_error(BMP388_HandleTypeDef *dev)
{
    uint8_t error = 0U;
    HAL_StatusTypeDef status = BMP388_ReadError(dev, &error);

    if (status != HAL_OK)
    {
        return status;
    }

    return ((error & BMP388_ERROR_MASK) == 0U) ? HAL_OK : HAL_ERROR;
}

/*
 * Read and convert the 21 factory calibration bytes stored in the BMP388 NVM.
 * These coefficients are unique to each sensor and must be applied to every
 * raw temperature and pressure reading.
 */
static HAL_StatusTypeDef bmp388_read_calibration(BMP388_HandleTypeDef *dev)
{
    uint8_t raw[BMP388_CALIB_DATA_LENGTH];
    HAL_StatusTypeDef status;

    status = bmp388_read_bytes(dev,
                               BMP388_CALIB_DATA_START,
                               raw,
                               (uint16_t)sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Raw NVM coefficient types and byte order are defined by the datasheet. */
    const uint16_t nvm_t1 = bmp388_u16_le(&raw[0]);
    const uint16_t nvm_t2 = bmp388_u16_le(&raw[2]);
    const int8_t nvm_t3 = (int8_t)raw[4];

    const int16_t nvm_p1 = (int16_t)bmp388_u16_le(&raw[5]);
    const int16_t nvm_p2 = (int16_t)bmp388_u16_le(&raw[7]);
    const int8_t nvm_p3 = (int8_t)raw[9];
    const int8_t nvm_p4 = (int8_t)raw[10];
    const uint16_t nvm_p5 = bmp388_u16_le(&raw[11]);
    const uint16_t nvm_p6 = bmp388_u16_le(&raw[13]);
    const int8_t nvm_p7 = (int8_t)raw[15];
    const int8_t nvm_p8 = (int8_t)raw[16];
    const int16_t nvm_p9 = (int16_t)bmp388_u16_le(&raw[17]);
    const int8_t nvm_p10 = (int8_t)raw[19];
    const int8_t nvm_p11 = (int8_t)raw[20];

    /* Convert NVM integers into the floating-point coefficients in Appendix 9. */
    dev->calib.par_t1 = (float)nvm_t1 * 256.0f;                         /* / 2^-8  */
    dev->calib.par_t2 = (float)nvm_t2 / 1073741824.0f;                 /* / 2^30  */
    dev->calib.par_t3 = (float)nvm_t3 / 281474976710656.0f;            /* / 2^48  */

    dev->calib.par_p1 = ((float)nvm_p1 - 16384.0f) / 1048576.0f;       /* / 2^20  */
    dev->calib.par_p2 = ((float)nvm_p2 - 16384.0f) / 536870912.0f;     /* / 2^29  */
    dev->calib.par_p3 = (float)nvm_p3 / 4294967296.0f;                 /* / 2^32  */
    dev->calib.par_p4 = (float)nvm_p4 / 137438953472.0f;               /* / 2^37  */
    dev->calib.par_p5 = (float)nvm_p5 * 8.0f;                          /* / 2^-3  */
    dev->calib.par_p6 = (float)nvm_p6 / 64.0f;                         /* / 2^6   */
    dev->calib.par_p7 = (float)nvm_p7 / 256.0f;                        /* / 2^8   */
    dev->calib.par_p8 = (float)nvm_p8 / 32768.0f;                      /* / 2^15  */
    dev->calib.par_p9 = (float)nvm_p9 / 281474976710656.0f;            /* / 2^48  */
    dev->calib.par_p10 = (float)nvm_p10 / 281474976710656.0f;          /* / 2^48  */
    dev->calib.par_p11 = (float)nvm_p11 / 36893488147419103232.0f;    /* / 2^65  */

    dev->calib.t_lin = 0.0f;
    dev->calib_valid = 1U;

    return HAL_OK;
}

/*
 * Compensate raw temperature and update t_lin. Pressure compensation depends
 * on t_lin, so this function must always run before pressure compensation.
 */
static float bmp388_compensate_temperature(
    uint32_t uncomp_temp,
    BMP388_calib_data *calib)
{
    const float partial_data1 = (float)uncomp_temp - calib->par_t1;
    const float partial_data2 = partial_data1 * calib->par_t2;

    calib->t_lin = partial_data2 +
                   (partial_data1 * partial_data1) * calib->par_t3;

    return calib->t_lin;
}

/* Convert the raw pressure ADC count into compensated pressure in pascals. */
static float bmp388_compensate_pressure(
    uint32_t uncomp_press,
    const BMP388_calib_data *calib)
{
    const float pressure = (float)uncomp_press;
    const float t_lin = calib->t_lin;
    const float t_lin2 = t_lin * t_lin;
    const float t_lin3 = t_lin2 * t_lin;

    const float partial_out1 = calib->par_p5 +
                               calib->par_p6 * t_lin +
                               calib->par_p7 * t_lin2 +
                               calib->par_p8 * t_lin3;

    const float partial_out2 = pressure *
                              (calib->par_p1 +
                               calib->par_p2 * t_lin +
                               calib->par_p3 * t_lin2 +
                               calib->par_p4 * t_lin3);

    const float pressure2 = pressure * pressure;
    const float pressure3 = pressure2 * pressure;
    const float partial_data4 = pressure2 *
                                (calib->par_p9 + calib->par_p10 * t_lin) +
                                pressure3 * calib->par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

/* Read the device identification register. A BMP388 should return 0x50. */
HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id)
{
    return bmp388_read_u8(dev, BMP388_CHIP_ID_REG, chip_id);
}

/*
 * Read STATUS (0x03):
 *   bit 4 = command decoder ready
 *   bit 5 = unread pressure sample available
 *   bit 6 = unread temperature sample available
 */
HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status)
{
    return bmp388_read_u8(dev, BMP388_STATUS_REG, status);
}

/*
 * Read ERR_REG (0x02):
 *   bit 0 = fatal sensor error
 *   bit 1 = command execution error
 *   bit 2 = invalid sensor configuration
 */
HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error)
{
    return bmp388_read_u8(dev, BMP388_ERR_REG, error);
}

/*
 * Initialize the BMP388 in continuous normal mode.
 *
 * Sequence:
 *   1. Confirm that an I2C device responds.
 *   2. Check the chip ID.
 *   3. Soft-reset the sensor and wait for its command decoder.
 *   4. Read the factory calibration coefficients.
 *   5. Write and verify oversampling, ODR, IIR, and power settings.
 *   6. Check ERR_REG and wait for the first complete sample.
 *
 * The function leaves calib_valid cleared whenever initialization fails, so a
 * caller cannot accidentally use partially initialized compensation data.
 */
HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->hi2c == NULL || dev->addr_7bit > 0x7FU)
    {
        return HAL_ERROR;
    }

    /* Invalidate all data that belongs to an earlier initialization. */
    dev->chip_id = 0U;
    dev->calib_valid = 0U;
    dev->reference_pressure_pa = 0.0f;
    dev->reference_pressure_valid = 0U;

    status = HAL_I2C_IsDeviceReady(dev->hi2c,
                                   BMP388_ADDR(dev),
                                   3U,
                                   BMP388_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    if (dev->chip_id != BMP388_CHIP_ID_VALUE)
    {
        status = HAL_ERROR;
        goto init_failed;
    }

    /* Reset restores all user configuration registers to their defaults. */
    status = bmp388_write_u8(dev, BMP388_CMD, BMP388_CMD_SOFT_RESET);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Allow the I2C interface to become accessible before polling STATUS. */
    HAL_Delay(5U);

    status = bmp388_wait_for_status(dev,
                                    BMP388_STATUS_CMD_READY_MASK,
                                    BMP388_STATUS_CMD_READY_MASK,
                                    BMP388_RESET_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Confirm that communication still works and the correct device returned. */
    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    if (dev->chip_id != BMP388_CHIP_ID_VALUE)
    {
        status = HAL_ERROR;
        goto init_failed;
    }

    status = bmp388_check_sensor_error(dev);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Calibration must be loaded before any raw measurement is compensated. */
    status = bmp388_read_calibration(dev);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Pressure x4 and temperature x1. */
    status = bmp388_write_u8(dev, BMP388_OSR, BMP388_CONFIG_OSR_VALUE);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* 50 Hz: one new pressure/temperature pair every 20 ms. */
    status = bmp388_write_u8(dev, BMP388_ODR, BMP388_CONFIG_ODR_VALUE);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* IIR coefficient 3 for modest smoothing with limited response delay. */
    status = bmp388_write_u8(dev, BMP388_CONFIG, BMP388_CONFIG_IIR_VALUE);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Enable pressure, temperature, and normal continuous-measurement mode. */
    status = bmp388_write_u8(dev,
                             BMP388_PWR_CTRL,
                             BMP388_CONFIG_PWR_CTRL_VALUE);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Read the registers back to catch a failed or incomplete configuration. */
    status = bmp388_verify_register(dev,
                                    BMP388_OSR,
                                    BMP388_CONFIG_OSR_VALUE,
                                    BMP388_OSR_VERIFY_MASK);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    status = bmp388_verify_register(dev,
                                    BMP388_ODR,
                                    BMP388_CONFIG_ODR_VALUE,
                                    BMP388_ODR_VERIFY_MASK);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    status = bmp388_verify_register(dev,
                                    BMP388_CONFIG,
                                    BMP388_CONFIG_IIR_VALUE,
                                    BMP388_CONFIG_VERIFY_MASK);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    status = bmp388_verify_register(dev,
                                    BMP388_PWR_CTRL,
                                    BMP388_CONFIG_PWR_CTRL_VALUE,
                                    BMP388_PWR_CTRL_VERIFY_MASK);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    status = bmp388_check_sensor_error(dev);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    /* Wait until both pressure and temperature from the first cycle are ready. */
    status = bmp388_wait_for_status(dev,
                                    BMP388_STATUS_DATA_READY_MASK,
                                    BMP388_STATUS_DATA_READY_MASK,
                                    BMP388_DATA_READY_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    return HAL_OK;

init_failed:
    dev->calib_valid = 0U;
    dev->reference_pressure_pa = 0.0f;
    dev->reference_pressure_valid = 0U;
    return status;
}

/*
 * Read the six raw data bytes immediately.
 *
 * This low-level function does not wait for data-ready. It is useful when the
 * caller already synchronizes reads with an interrupt or STATUS check. Most
 * application code should call BMP388_ReadData(), which waits for fresh data.
 */
HAL_StatusTypeDef BMP388_ReadRawData(
    BMP388_HandleTypeDef *dev,
    uint32_t raw_data[BMP388_DATA_COUNT])
{
    uint8_t raw[6];
    HAL_StatusTypeDef status;

    if (dev == NULL || raw_data == NULL)
    {
        return HAL_ERROR;
    }

    /* One burst read keeps pressure and temperature in the same sample. */
    status = bmp388_read_bytes(dev,
                               BMP388_DATA_0,
                               raw,
                               (uint16_t)sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Registers 0x04..0x06 contain an unsigned 24-bit pressure value. */
    raw_data[BMP388_PRESSURE] = ((uint32_t)raw[2] << 16U) |
                                ((uint32_t)raw[1] << 8U) |
                                (uint32_t)raw[0];

    /* Registers 0x07..0x09 contain an unsigned 24-bit temperature value. */
    raw_data[BMP388_TEMPERATURE] = ((uint32_t)raw[5] << 16U) |
                                   ((uint32_t)raw[4] << 8U) |
                                   (uint32_t)raw[3];

    return HAL_OK;
}

/*
 * Wait for and return one fresh compensated sample.
 *
 * data[BMP388_PRESSURE]    = pressure in pascals
 * data[BMP388_TEMPERATURE] = sensor temperature in degrees Celsius
 *
 * This function blocks for at most BMP388_DATA_READY_TIMEOUT_MS. At the default
 * 50 Hz ODR, a normal wait is no more than one 20 ms sample period.
 */
HAL_StatusTypeDef BMP388_ReadData(
    BMP388_HandleTypeDef *dev,
    float data[BMP388_DATA_COUNT])
{
    uint32_t raw_data[BMP388_DATA_COUNT];
    HAL_StatusTypeDef status;

    if (dev == NULL || data == NULL || dev->calib_valid == 0U)
    {
        return HAL_ERROR;
    }

    /* Avoid returning the same register contents repeatedly between samples. */
    status = bmp388_wait_for_status(dev,
                                    BMP388_STATUS_DATA_READY_MASK,
                                    BMP388_STATUS_DATA_READY_MASK,
                                    BMP388_DATA_READY_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP388_ReadRawData(dev, raw_data);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Pressure compensation uses t_lin, which is calculated from temperature. */
    data[BMP388_TEMPERATURE] =
        bmp388_compensate_temperature(raw_data[BMP388_TEMPERATURE],
                                      &dev->calib);

    data[BMP388_PRESSURE] =
        bmp388_compensate_pressure(raw_data[BMP388_PRESSURE],
                                   &dev->calib);

    if (!isfinite(data[BMP388_TEMPERATURE]) ||
        !isfinite(data[BMP388_PRESSURE]) ||
        data[BMP388_PRESSURE] <= 0.0f)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*
 * Convert pressure to altitude using the International Standard Atmosphere
 * approximation:
 *
 *     h = 44330 * (1 - (P / P0)^(1 / 5.255))
 *
 * P0 should normally be the average pressure measured while the rocket is
 * stationary on the launch pad. The returned altitude is then relative to the
 * launch pad rather than relative to sea level.
 */
float BMP388_PressureToAltitude(
    float pressure_pa,
    float reference_pressure_pa)
{
    if (!isfinite(pressure_pa) ||
        !isfinite(reference_pressure_pa) ||
        pressure_pa <= 0.0f ||
        reference_pressure_pa <= 0.0f)
    {
        return NAN;
    }

    return 44330.0f *
           (1.0f - powf(pressure_pa / reference_pressure_pa,
                        0.1902949572f));
}

/* Manually supply the pressure that represents zero relative altitude. */
HAL_StatusTypeDef BMP388_SetReferencePressure(
    BMP388_HandleTypeDef *dev,
    float reference_pressure_pa)
{
    if (dev == NULL ||
        !isfinite(reference_pressure_pa) ||
        reference_pressure_pa <= 0.0f)
    {
        return HAL_ERROR;
    }

    dev->reference_pressure_pa = reference_pressure_pa;
    dev->reference_pressure_valid = 1U;
    return HAL_OK;
}

/*
 * Average several fresh pressure samples to establish launch-pad pressure.
 *
 * BMP388_ReadData() already waits for a new conversion before each sample, so
 * sample_delay_ms may be zero. A nonzero delay intentionally spreads the
 * calibration over a longer time, which can further average slow pressure
 * fluctuations. The rocket must remain stationary during this function.
 */
HAL_StatusTypeDef BMP388_CalibrateAltitudeReference(
    BMP388_HandleTypeDef *dev,
    uint16_t sample_count,
    uint32_t sample_delay_ms)
{
    float data[BMP388_DATA_COUNT];
    double pressure_sum = 0.0;
    uint16_t sample;
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->calib_valid == 0U || sample_count == 0U)
    {
        return HAL_ERROR;
    }

    /* Do not leave an old calibration marked valid if this attempt fails. */
    dev->reference_pressure_pa = 0.0f;
    dev->reference_pressure_valid = 0U;

    for (sample = 0U; sample < sample_count; ++sample)
    {
        status = BMP388_ReadData(dev, data);
        if (status != HAL_OK)
        {
            return status;
        }

        pressure_sum += (double)data[BMP388_PRESSURE];

        if (sample_delay_ms > 0U && (sample + 1U) < sample_count)
        {
            HAL_Delay(sample_delay_ms);
        }
    }

    dev->reference_pressure_pa =
        (float)(pressure_sum / (double)sample_count);

    if (!isfinite(dev->reference_pressure_pa) ||
        dev->reference_pressure_pa <= 0.0f)
    {
        dev->reference_pressure_pa = 0.0f;
        return HAL_ERROR;
    }

    dev->reference_pressure_valid = 1U;
    return HAL_OK;
}

/* Read one fresh pressure sample and return altitude relative to reference. */
HAL_StatusTypeDef BMP388_ReadAltitude(
    BMP388_HandleTypeDef *dev,
    float *altitude_m)
{
    float data[BMP388_DATA_COUNT];
    HAL_StatusTypeDef status;

    if (dev == NULL || altitude_m == NULL ||
        dev->reference_pressure_valid == 0U)
    {
        return HAL_ERROR;
    }

    status = BMP388_ReadData(dev, data);
    if (status != HAL_OK)
    {
        return status;
    }

    *altitude_m = BMP388_PressureToAltitude(
        data[BMP388_PRESSURE],
        dev->reference_pressure_pa);

    return isfinite(*altitude_m) ? HAL_OK : HAL_ERROR;
}
