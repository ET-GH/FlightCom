#include "sx1280_port.h"

//keeping SPI in a different file, you don't need to edit this

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef SX1280_PortInit(void)
{
    SX1280_PortCsHigh();
    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

void SX1280_PortCsLow(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

void SX1280_PortCsHigh(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SX1280_PortReset(void)
{
    SX1280_PortCsHigh();

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

HAL_StatusTypeDef SX1280_PortWaitBusyLow(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef SX1280_PortSpiTxRx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100);
}
