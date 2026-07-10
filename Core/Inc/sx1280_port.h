#ifndef SX1280_PORT_H
#define SX1280_PORT_H

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef SX1280_PortInit(void);
HAL_StatusTypeDef SX1280_PortWaitBusyLow(uint32_t timeout_ms);
HAL_StatusTypeDef SX1280_PortSpiTxRx(const uint8_t *tx, uint8_t *rx, uint16_t len);

void SX1280_PortCsLow(void);
void SX1280_PortCsHigh(void);
void SX1280_PortReset(void);

#endif
