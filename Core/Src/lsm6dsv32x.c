#include "lsm6dsv32x.h"

#define LSM6DSV32X_I2C_TIMEOUT_MS    100U
#define LSM6DSV32X_ADDR(dev)         ((uint16_t)((dev)->addr_7bit << 1))

// Helpers
static HAL_StatusTypeDef lsm6_write_u8(LSM6DSV32X_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             LSM6DSV32X_ADDR(dev),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             LSM6DSV32X_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef lsm6_read_u8(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t reg,
    uint8_t *value
)
{
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            LSM6DSV32X_ADDR(dev),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            LSM6DSV32X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LSM6DSV32X_ReadWhoAmI(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t *who_am_i
)
{
    if (who_am_i == NULL)
    {
        return HAL_ERROR;
    }

    return lsm6_read_u8(dev, LSM6DSV32X_WHO_AM_I_REG, who_am_i);
}

HAL_StatusTypeDef LSM6DSV32X_ReadStatus(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t *status
)
{
    if (status == NULL)
    {
        return HAL_ERROR;
    }

    return lsm6_read_u8(dev, LSM6DSV32X_STATUS_REG, status);
}

static HAL_StatusTypeDef lsm6_read_bytes(LSM6DSV32X_HandleTypeDef *dev, uint8_t start_reg, uint8_t *buffer, uint16_t len)
{
    if (dev == NULL || dev->hi2c == NULL || buffer == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            LSM6DSV32X_ADDR(dev),
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            LSM6DSV32X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LSM6DSV32X_Init(LSM6DSV32X_HandleTypeDef *dev)
{
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(dev->hi2c, LSM6DSV32X_ADDR(dev), 3U, LSM6DSV32X_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = LSM6DSV32X_ReadWhoAmI(dev, &dev->who_am_i);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->who_am_i != LSM6DSV32X_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    // Software reset: CTRL3 bit0 = SW_RESET.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL3, 0x01U);
    if (status != HAL_OK)
    {
        return status;
    }

    uint32_t reset_start = HAL_GetTick();
    uint8_t ctrl3 = 0x01U;

    do
    {
        HAL_Delay(1U);

        status = lsm6_read_u8(dev, LSM6DSV32X_CTRL3, &ctrl3);
        if (status != HAL_OK)
        {
            return status;
        }

        if ((ctrl3 & 0x01U) == 0U)
        {
            break;
        }

    } while ((HAL_GetTick() - reset_start) < 100U);

    if ((ctrl3 & 0x01U) != 0U)
    {
        return HAL_TIMEOUT;
    }

    // CTRL3: BDU=1, IF_INC=1. Reserved bits remain 0.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL3, 0x44U);
    if (status != HAL_OK) { return status; }

    // CTRL8: bit2 must be 1. FS_XL[1:0] = 11 -> +/-32 g.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL8, 0b00000111);
    if (status != HAL_OK) { return status; }

    // CTRL6: FS_G[3:0] = 1100 -> +/-4000 dps. LPF bits left 0.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL6, 0b00001100U);
    if (status != HAL_OK) { return status; }

    // CTRL1: accel high-performance mode, ODR_XL = 120 Hz.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL1, 0x06U);
    if (status != HAL_OK) { return status; }

    // CTRL2: gyro high-performance mode, ODR_G = 120 Hz.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL2, 0x06U);
    if (status != HAL_OK) { return status; }

    HAL_Delay(50U);
    return HAL_OK;
}

// reads and multiplies data by datasheet weights
HAL_StatusTypeDef LSM6DSV32X_ReadData(LSM6DSV32X_HandleTypeDef *dev, float data[LSM6DSV32X_DATA_COUNT])
{
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t raw[12];
    HAL_StatusTypeDef status = lsm6_read_bytes(dev, LSM6DSV32X_OUTX_L_G, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    data[LSM6DSV32X_GYRO_X]  = (float)(int16_t)((uint16_t)raw[1]  << 8 | (uint16_t)raw[0]) * 0.14;
    data[LSM6DSV32X_GYRO_Y]  = (float)(int16_t)((uint16_t)raw[3]  << 8 | (uint16_t)raw[2]) * -0.14;
    data[LSM6DSV32X_GYRO_Z]  = (float)(int16_t)((uint16_t)raw[5]  << 8 | (uint16_t)raw[4]) * 0.14;
    data[LSM6DSV32X_ACCEL_X] = (float)(int16_t)((uint16_t)raw[7]  << 8 | (uint16_t)raw[6]) * 0.000976;
    data[LSM6DSV32X_ACCEL_Y] = (float)(int16_t)((uint16_t)raw[9]  << 8 | (uint16_t)raw[8]) * -0.000976;
    data[LSM6DSV32X_ACCEL_Z] = (float)(int16_t)((uint16_t)raw[11] << 8 | (uint16_t)raw[10]) * 0.000976;

    return HAL_OK;
}
