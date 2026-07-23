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
 * Packet transmission wrappers. These queue complete AMBAR packets.
 */
bool USBComm_SendTelemetry(const AmbarHilUsbTelemetry *telemetry);
bool USBComm_SendEvent(const AmbarHilUsbEvent *event);
bool USBComm_SendHeartbeat(const AmbarHilUsbHeartbeat *heartbeat);
bool USBComm_SendActuatorStatus(
    const AmbarHilUsbActuatorStatus *status);

/*
 * Returns AMBAR transport statistics.
 */
AmbarHilUsbStats USBComm_GetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMM_H */
