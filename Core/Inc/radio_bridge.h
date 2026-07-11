#ifndef RADIO_BRIDGE_H
#define RADIO_BRIDGE_H

#include "main.h"
#include "behavior.h"
#include <stdint.h>
#include <string.h>

HAL_StatusTypeDef RadioBridge_Init(void);
void RadioBridge_Task(const volatile BehaviorTelemetry_t *telemetry, uint32_t now_ms);
void RadioBridge_RequestSnapshot(void);

#endif
