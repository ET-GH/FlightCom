/**
 * @file w25q64.h
 * @brief Simple blocking HAL driver for Winbond W25Q64JV SPI NOR flash.
 *
 * Drop into Core/Inc.
 */

#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define W25Q64_FLASH_SIZE_BYTES      (8UL * 1024UL * 1024UL)
#define W25Q64_PAGE_SIZE_BYTES       256UL
#define W25Q64_SECTOR_SIZE_BYTES     4096UL
#define W25Q64_BLOCK32_SIZE_BYTES    (32UL * 1024UL)
#define W25Q64_BLOCK64_SIZE_BYTES    (64UL * 1024UL)

typedef enum
{
    W25Q64_OK = 0,
    W25Q64_ERROR = 1,
    W25Q64_TIMEOUT = 2,
    W25Q64_BAD_ID = 3,
    W25Q64_INVALID_ARGUMENT = 4,
    W25Q64_OUT_OF_RANGE = 5
} W25Q64_Result_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t spi_timeout_ms;
    uint32_t program_timeout_ms;
    uint32_t erase_timeout_ms;
} W25Q64_HandleTypeDef;

void W25Q64_Attach(W25Q64_HandleTypeDef *dev,
                   SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port,
                   uint16_t cs_pin);

W25Q64_Result_t W25Q64_Init(W25Q64_HandleTypeDef *dev);

W25Q64_Result_t W25Q64_ReadJEDECID(W25Q64_HandleTypeDef *dev,
                                   uint8_t id[3]);

W25Q64_Result_t W25Q64_Read(W25Q64_HandleTypeDef *dev,
                            uint32_t addr,
                            uint8_t *data,
                            uint32_t len);

W25Q64_Result_t W25Q64_Write(W25Q64_HandleTypeDef *dev,
                             uint32_t addr,
                             const uint8_t *data,
                             uint32_t len);

W25Q64_Result_t W25Q64_EraseSector(W25Q64_HandleTypeDef *dev,
                                   uint32_t sector_addr);

W25Q64_Result_t W25Q64_EraseRange(W25Q64_HandleTypeDef *dev,
                                  uint32_t start_addr,
                                  uint32_t length_bytes);

W25Q64_Result_t W25Q64_WaitReady(W25Q64_HandleTypeDef *dev,
                                 uint32_t timeout_ms);

W25Q64_Result_t W25Q64_ReadStatus1(W25Q64_HandleTypeDef *dev,
                                   uint8_t *status1);

W25Q64_Result_t W25Q64_PowerDown(W25Q64_HandleTypeDef *dev);
W25Q64_Result_t W25Q64_ReleasePowerDown(W25Q64_HandleTypeDef *dev);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
