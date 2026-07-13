/**
 * NOT TESTED AND WAS QUICKLY GENERATED
 * It mainly was so that I have at least something to work with for the USB
 * but since USB never worked, I was never able to actually open and work with this.
 * I didn't want to research the datasheet if I couldn't have the USB working.
 */

#include "w25q64.h"

#define W25Q64_CMD_WRITE_ENABLE       0x06U
#define W25Q64_CMD_WRITE_DISABLE      0x04U
#define W25Q64_CMD_READ_STATUS1       0x05U
#define W25Q64_CMD_READ_DATA          0x03U
#define W25Q64_CMD_PAGE_PROGRAM       0x02U
#define W25Q64_CMD_SECTOR_ERASE       0x20U
#define W25Q64_CMD_JEDEC_ID           0x9FU
#define W25Q64_CMD_POWER_DOWN         0xB9U
#define W25Q64_CMD_RELEASE_POWER_DOWN 0xABU

#define W25Q64_STATUS1_BUSY           0x01U

static uint32_t W25Q64_TimeoutOrDefault(uint32_t configured, uint32_t fallback)
{
    return (configured == 0U) ? fallback : configured;
}

static void W25Q64_CS_Low(W25Q64_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void W25Q64_CS_High(W25Q64_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static W25Q64_Result_t W25Q64_CheckArgs(W25Q64_HandleTypeDef *dev)
{
    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    return W25Q64_OK;
}

static W25Q64_Result_t W25Q64_FromHAL(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return W25Q64_OK;
    }

    if (status == HAL_TIMEOUT)
    {
        return W25Q64_TIMEOUT;
    }

    return W25Q64_ERROR;
}

static W25Q64_Result_t W25Q64_Tx(W25Q64_HandleTypeDef *dev,
                                 const uint8_t *data,
                                 uint16_t len)
{
    uint32_t timeout = W25Q64_TimeoutOrDefault(dev->spi_timeout_ms, 100U);
    return W25Q64_FromHAL(HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, timeout));
}

static W25Q64_Result_t W25Q64_Rx(W25Q64_HandleTypeDef *dev,
                                 uint8_t *data,
                                 uint16_t len)
{
    uint32_t timeout = W25Q64_TimeoutOrDefault(dev->spi_timeout_ms, 100U);
    return W25Q64_FromHAL(HAL_SPI_Receive(dev->hspi, data, len, timeout));
}

static W25Q64_Result_t W25Q64_CommandOnly(W25Q64_HandleTypeDef *dev, uint8_t command)
{
    W25Q64_Result_t result;

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, &command, 1U);
    W25Q64_CS_High(dev);

    return result;
}

static W25Q64_Result_t W25Q64_WriteEnable(W25Q64_HandleTypeDef *dev)
{
    return W25Q64_CommandOnly(dev, W25Q64_CMD_WRITE_ENABLE);
}

void W25Q64_Attach(W25Q64_HandleTypeDef *dev,
                   SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port,
                   uint16_t cs_pin)
{
    if (dev == NULL)
    {
        return;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->spi_timeout_ms = 100U;
    dev->program_timeout_ms = 100U;
    dev->erase_timeout_ms = 1000U;
}

W25Q64_Result_t W25Q64_ReadStatus1(W25Q64_HandleTypeDef *dev, uint8_t *status1)
{
    uint8_t command = W25Q64_CMD_READ_STATUS1;
    W25Q64_Result_t result;

    if (status1 == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);

    result = W25Q64_Tx(dev, &command, 1U);
    if (result == W25Q64_OK)
    {
        result = W25Q64_Rx(dev, status1, 1U);
    }

    W25Q64_CS_High(dev);

    return result;
}

W25Q64_Result_t W25Q64_WaitReady(W25Q64_HandleTypeDef *dev, uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint8_t status1 = 0U;
    W25Q64_Result_t result;

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    start_ms = HAL_GetTick();

    do
    {
        result = W25Q64_ReadStatus1(dev, &status1);
        if (result != W25Q64_OK)
        {
            return result;
        }

        if ((status1 & W25Q64_STATUS1_BUSY) == 0U)
        {
            return W25Q64_OK;
        }
    }
    while ((HAL_GetTick() - start_ms) < timeout_ms);

    return W25Q64_TIMEOUT;
}

W25Q64_Result_t W25Q64_ReadJEDECID(W25Q64_HandleTypeDef *dev, uint8_t id[3])
{
    uint8_t command = W25Q64_CMD_JEDEC_ID;
    W25Q64_Result_t result;

    if (id == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);

    result = W25Q64_Tx(dev, &command, 1U);
    if (result == W25Q64_OK)
    {
        result = W25Q64_Rx(dev, id, 3U);
    }

    W25Q64_CS_High(dev);

    return result;
}

W25Q64_Result_t W25Q64_Init(W25Q64_HandleTypeDef *dev)
{
    uint8_t id[3] = {0U, 0U, 0U};
    W25Q64_Result_t result;

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_High(dev);
    HAL_Delay(2U);

    result = W25Q64_ReadJEDECID(dev, id);
    if (result != W25Q64_OK)
    {
        return result;
    }

    /*
     * Winbond manufacturer = 0xEF.
     * Common W25Q64JV JEDEC IDs:
     *   EF 40 17  for W25Q64JV-IQ/JQ
     *   EF 70 17  for W25Q64JV-IM/JM
     */
    if ((id[0] != 0xEFU) || (id[2] != 0x17U))
    {
        return W25Q64_BAD_ID;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_Read(W25Q64_HandleTypeDef *dev,
                            uint32_t addr,
                            uint8_t *data,
                            uint32_t len)
{
    uint8_t command[4];
    W25Q64_Result_t result;

    if ((data == NULL) && (len > 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if ((addr > W25Q64_FLASH_SIZE_BYTES) ||
        (len > W25Q64_FLASH_SIZE_BYTES) ||
        ((addr + len) > W25Q64_FLASH_SIZE_BYTES))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    if (len == 0U)
    {
        return W25Q64_OK;
    }

    command[0] = W25Q64_CMD_READ_DATA;
    command[1] = (uint8_t)(addr >> 16);
    command[2] = (uint8_t)(addr >> 8);
    command[3] = (uint8_t)(addr);

    W25Q64_CS_Low(dev);

    result = W25Q64_Tx(dev, command, sizeof(command));
    if (result == W25Q64_OK)
    {
        /*
         * HAL_SPI_Receive takes uint16_t length on STM32 HAL.
         * Split large reads into safe chunks.
         */
        while ((result == W25Q64_OK) && (len > 0U))
        {
            uint16_t chunk = (len > 65535U) ? 65535U : (uint16_t)len;
            result = W25Q64_Rx(dev, data, chunk);
            data += chunk;
            len -= chunk;
        }
    }

    W25Q64_CS_High(dev);

    return result;
}

W25Q64_Result_t W25Q64_EraseSector(W25Q64_HandleTypeDef *dev, uint32_t sector_addr)
{
    uint8_t command[4];
    uint32_t aligned_addr;
    W25Q64_Result_t result;
    uint32_t timeout;

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (sector_addr >= W25Q64_FLASH_SIZE_BYTES)
    {
        return W25Q64_OUT_OF_RANGE;
    }

    aligned_addr = sector_addr & ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);

    result = W25Q64_WriteEnable(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    command[0] = W25Q64_CMD_SECTOR_ERASE;
    command[1] = (uint8_t)(aligned_addr >> 16);
    command[2] = (uint8_t)(aligned_addr >> 8);
    command[3] = (uint8_t)(aligned_addr);

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, command, sizeof(command));
    W25Q64_CS_High(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    timeout = W25Q64_TimeoutOrDefault(dev->erase_timeout_ms, 1000U);
    return W25Q64_WaitReady(dev, timeout);
}

W25Q64_Result_t W25Q64_EraseRange(W25Q64_HandleTypeDef *dev,
                                  uint32_t start_addr,
                                  uint32_t length_bytes)
{
    uint32_t addr;
    uint32_t end_addr;
    W25Q64_Result_t result;

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (length_bytes == 0U)
    {
        return W25Q64_OK;
    }

    if ((start_addr >= W25Q64_FLASH_SIZE_BYTES) ||
        (length_bytes > W25Q64_FLASH_SIZE_BYTES) ||
        ((start_addr + length_bytes) > W25Q64_FLASH_SIZE_BYTES))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    addr = start_addr & ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);
    end_addr = start_addr + length_bytes;

    while (addr < end_addr)
    {
        result = W25Q64_EraseSector(dev, addr);
        if (result != W25Q64_OK)
        {
            return result;
        }

        addr += W25Q64_SECTOR_SIZE_BYTES;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_Write(W25Q64_HandleTypeDef *dev,
                             uint32_t addr,
                             const uint8_t *data,
                             uint32_t len)
{
    W25Q64_Result_t result;

    if ((data == NULL) && (len > 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if ((addr > W25Q64_FLASH_SIZE_BYTES) ||
        (len > W25Q64_FLASH_SIZE_BYTES) ||
        ((addr + len) > W25Q64_FLASH_SIZE_BYTES))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    while (len > 0U)
    {
        uint8_t command[4];
        uint32_t page_offset = addr % W25Q64_PAGE_SIZE_BYTES;
        uint32_t room_in_page = W25Q64_PAGE_SIZE_BYTES - page_offset;
        uint16_t chunk = (len < room_in_page) ? (uint16_t)len : (uint16_t)room_in_page;
        uint32_t timeout = W25Q64_TimeoutOrDefault(dev->program_timeout_ms, 100U);

        result = W25Q64_WriteEnable(dev);
        if (result != W25Q64_OK)
        {
            return result;
        }

        command[0] = W25Q64_CMD_PAGE_PROGRAM;
        command[1] = (uint8_t)(addr >> 16);
        command[2] = (uint8_t)(addr >> 8);
        command[3] = (uint8_t)(addr);

        W25Q64_CS_Low(dev);

        result = W25Q64_Tx(dev, command, sizeof(command));
        if (result == W25Q64_OK)
        {
            result = W25Q64_Tx(dev, data, chunk);
        }

        W25Q64_CS_High(dev);

        if (result != W25Q64_OK)
        {
            return result;
        }

        result = W25Q64_WaitReady(dev, timeout);
        if (result != W25Q64_OK)
        {
            return result;
        }

        addr += chunk;
        data += chunk;
        len -= chunk;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_PowerDown(W25Q64_HandleTypeDef *dev)
{
    W25Q64_Result_t result = W25Q64_CommandOnly(dev, W25Q64_CMD_POWER_DOWN);
    HAL_Delay(1U);
    return result;
}

W25Q64_Result_t W25Q64_ReleasePowerDown(W25Q64_HandleTypeDef *dev)
{
    W25Q64_Result_t result = W25Q64_CommandOnly(dev, W25Q64_CMD_RELEASE_POWER_DOWN);
    HAL_Delay(1U);
    return result;
}
