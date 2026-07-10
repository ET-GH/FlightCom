#ifndef RADIO_BRIDGE_H
#define RADIO_BRIDGE_H

#include "main.h"
#include <stdint.h>
#include <string.h>

HAL_StatusTypeDef RadioBridge_Init(void);
void RadioBridge_Task(void);
HAL_StatusTypeDef RadioBridge_SendText(const char *text);
HAL_StatusTypeDef PayloadPipeline(uint16_t *IMU,
                                  uint32_t *Baro,
								  uint16_t *Magnet,
								  uint16_t *Calc,
								  const char *Message[3],
                                  uint8_t deployment_state);

#endif
