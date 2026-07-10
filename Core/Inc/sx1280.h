#ifndef SX1280_H
#define SX1280_H

#include "main.h"
#include <stdint.h>

#define SX1280_MAX_PAYLOAD_LEN 200

HAL_StatusTypeDef SX1280_InitLoRa(void);
HAL_StatusTypeDef SX1280_StartRxContinuous(void);
HAL_StatusTypeDef SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
HAL_StatusTypeDef SX1280_ReadPacketIfAvailable(uint8_t *data, uint8_t *len);

#endif
