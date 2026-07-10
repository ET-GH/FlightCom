#ifndef USB_COMM_H
#define USB_COMM_H

#include "main.h"
#include <stdint.h>

void USBComm_Init(void);
void USBComm_Task(void);

void USBComm_Attach(void *cdc_acm_instance);
void USBComm_Detach(void);

HAL_StatusTypeDef USBComm_Write(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef USBComm_WriteString(const char *text);

#endif
