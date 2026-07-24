#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "ambar_hil_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initializes the AMBAR protocol transport.
 *
 * USBX and the STM32 PCD must already be configured separately.
 * Call this before HAL_PCD_Start().
 */
bool USBComm_Init(void);

/*
 * Services raw USBX CDC transfers and the AMBAR framing layer.
 * Call continuously from the cooperative main loop.
 */
void USBComm_Task(void);

/*
 * Called by the generated USBX CDC activation and deactivation callbacks.
 */
void USBComm_Attach(void *cdc_acm_instance);
void USBComm_Detach(void);

/*
 * True only after the host has configured the CDC ACM interface.
 */
bool USBComm_IsConnected(void);

/*
 * Application command callback.
 *
 * usb_comm.c validates the AMBAR framing and handles PING locally. main.c
 * supplies the strong implementation for application-owned commands such as
 * EXPORT_LOG and CANCEL_EXPORT.
 */
uint8_t USBComm_ExecuteCommand(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length,
    uint16_t command_sequence,
    uint16_t *detail);

/*
 * Packet transmission wrappers. These queue complete AMBAR packets.
 */
bool USBComm_SendTelemetry(const AmbarHilUsbTelemetry *telemetry);
bool USBComm_SendEvent(const AmbarHilUsbEvent *event);
bool USBComm_SendHeartbeat(const AmbarHilUsbHeartbeat *heartbeat);
bool USBComm_SendActuatorStatus(
    const AmbarHilUsbActuatorStatus *status);
bool USBComm_SendLogStatus(
    const AmbarHilUsbLogStatus *status);

/*
 * Returns AMBAR transport statistics.
 */
AmbarHilUsbStats USBComm_GetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMM_H */
