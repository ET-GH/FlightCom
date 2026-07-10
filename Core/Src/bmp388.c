#include "bmp388.h"

#include <stddef.h>

#define BMP388_I2C_TIMEOUT_MS            100U
#define BMP388_ADDR(dev)                 ((uint16_t)((dev)->addr_7bit << 1))

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

static uint16_t bmp388_u16_le(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

static int16_t bmp388_s16_le(const uint8_t *bytes)
{
    return (int16_t)bmp388_u16_le(bytes);
}

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

    /* Raw NVM coefficient types and byte order are defined by Table 23. */
    const uint16_t nvm_t1 = bmp388_u16_le(&raw[0]);
    const uint16_t nvm_t2 = bmp388_u16_le(&raw[2]);
    const int8_t nvm_t3 = (int8_t)raw[4];

    const int16_t nvm_p1 = bmp388_s16_le(&raw[5]);
    const int16_t nvm_p2 = bmp388_s16_le(&raw[7]);
    const int8_t nvm_p3 = (int8_t)raw[9];
    const int8_t nvm_p4 = (int8_t)raw[10];
    const uint16_t nvm_p5 = bmp388_u16_le(&raw[11]);
    const uint16_t nvm_p6 = bmp388_u16_le(&raw[13]);
    const int8_t nvm_p7 = (int8_t)raw[15];
    const int8_t nvm_p8 = (int8_t)raw[16];
    const int16_t nvm_p9 = bmp388_s16_le(&raw[17]);
    const int8_t nvm_p10 = (int8_t)raw[19];
    const int8_t nvm_p11 = (int8_t)raw[20];

    /* Convert NVM values to the floating-point coefficients in Appendix 9.1. */
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

static float bmp388_compensate_temperature(
    uint32_t uncomp_temp,
    BMP388_calib_data *calib)
{
    const float partial_data1 = (float)uncomp_temp - calib->par_t1;
    const float partial_data2 = partial_data1 * calib->par_t2;

    /* t_lin must be calculated before pressure compensation. */
    calib->t_lin = partial_data2 +
                   (partial_data1 * partial_data1) * calib->par_t3;

    return calib->t_lin;
}

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

HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id)
{
    return bmp388_read_u8(dev, BMP388_CHIP_ID_REG, chip_id);
}

HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status)
{
    return bmp388_read_u8(dev, BMP388_STATUS_REG, status);
}

HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error)
{
    return bmp388_read_u8(dev, BMP388_ERR_REG, error);
}

HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    dev->chip_id = 0U;
    dev->calib_valid = 0U;

    status = HAL_I2C_IsDeviceReady(dev->hi2c,
                                   BMP388_ADDR(dev),
                                   3U,
                                   BMP388_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->chip_id != BMP388_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    /* Soft reset, then allow the device to settle. */
    status = bmp388_write_u8(dev, BMP388_CMD, BMP388_CMD_SOFT_RESET);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10U);

    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->chip_id != BMP388_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    /* Calibration coefficients must be read after reset and before sampling. */
    status = bmp388_read_calibration(dev);
    if (status != HAL_OK)
    {
        return status;
    }

    /* OSR: pressure x4, temperature x2 -> osr_p=010, osr_t=001. */
    status = bmp388_write_u8(dev, BMP388_OSR, 0x0AU);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ODR: 50 Hz. */
    status = bmp388_write_u8(dev, BMP388_ODR, 0x02U);
    if (status != HAL_OK)
    {
        return status;
    }

    /* CONFIG: IIR coefficient 3. Use 0x00 to disable pressure smoothing. */
    status = bmp388_write_u8(dev, BMP388_CONFIG, 0x04U);
    if (status != HAL_OK)
    {
        return status;
    }

    /* PWR_CTRL: press_en=1, temp_en=1, normal mode=11. */
    status = bmp388_write_u8(dev, BMP388_PWR_CTRL, 0x33U);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(20U);
    return HAL_OK;
}

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

    /* One burst read keeps pressure and temperature from the same sample. */
    status = bmp388_read_bytes(dev,
                               BMP388_DATA_0,
                               raw,
                               (uint16_t)sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    raw_data[BMP388_PRESSURE] = ((uint32_t)raw[2] << 16) |
                                ((uint32_t)raw[1] << 8) |
                                (uint32_t)raw[0];

    raw_data[BMP388_TEMPERATURE] = ((uint32_t)raw[5] << 16) |
                                   ((uint32_t)raw[4] << 8) |
                                   (uint32_t)raw[3];

    return HAL_OK;
}

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

    status = BMP388_ReadRawData(dev, raw_data);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Temperature must be compensated first because pressure uses t_lin. */
    data[BMP388_TEMPERATURE] =
        bmp388_compensate_temperature(raw_data[BMP388_TEMPERATURE],
                                      &dev->calib);

    data[BMP388_PRESSURE] =
        bmp388_compensate_pressure(raw_data[BMP388_PRESSURE],
                                   &dev->calib);

    return HAL_OK;
}

float BMP388_PressureToAltitude(
    float pressure_pa,
    float reference_pressure_pa)
{
    /*
     * International Standard Atmosphere pressure-altitude approximation:
     * h = 44330 * (1 - (P / P0)^(1 / 5.255))
     *
     * When P0 is the launch-pad pressure, h is approximately relative to the
     * launch pad and is zero at the moment P0 is recorded.
     */
    if (pressure_pa <= 0.0f || reference_pressure_pa <= 0.0f)
    {
        return NAN;
    }

    return 44330.0f *
           (1.0f - powf(pressure_pa / reference_pressure_pa,
                        0.1902949572f));
}

HAL_StatusTypeDef BMP388_SetReferencePressure(
    BMP388_HandleTypeDef *dev,
    float reference_pressure_pa)
{
    if (dev == NULL || reference_pressure_pa <= 0.0f)
    {
        return HAL_ERROR;
    }

    dev->reference_pressure_pa = reference_pressure_pa;
    dev->reference_pressure_valid = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_CalibrateAltitudeReference(
    BMP388_HandleTypeDef *dev,
    uint16_t sample_count,
    uint32_t sample_delay_ms)
{
    float data[BMP388_DATA_COUNT];
    double pressure_sum = 0.0;
    uint16_t sample;
    HAL_StatusTypeDef status;

    if (dev == NULL || sample_count == 0U)
    {
        return HAL_ERROR;
    }

    for (sample = 0U; sample < sample_count; ++sample)
    {
        status = BMP388_ReadData(dev, data);
        if (status != HAL_OK)
        {
            dev->reference_pressure_valid = 0U;
            return status;
        }

        if (data[BMP388_PRESSURE] <= 0.0f)
        {
            dev->reference_pressure_valid = 0U;
            return HAL_ERROR;
        }

        pressure_sum += (double)data[BMP388_PRESSURE];

        if (sample_delay_ms > 0U && (sample + 1U) < sample_count)
        {
            HAL_Delay(sample_delay_ms);
        }
    }

    dev->reference_pressure_pa =
        (float)(pressure_sum / (double)sample_count);
    dev->reference_pressure_valid = 1U;

    return HAL_OK;
}

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

    if (isnan(*altitude_m))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
