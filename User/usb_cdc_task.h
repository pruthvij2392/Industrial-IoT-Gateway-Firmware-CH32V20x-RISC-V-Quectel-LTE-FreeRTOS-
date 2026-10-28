#ifndef __USB_CDC_TASK_H
#define __USB_CDC_TASK_H

#include "ch32v20x.h"
#include "FreeRTOS.h"
#include "queue.h"

/**
 * @brief FreeRTOS Queue for receiving bytes from USB Endpoint 2 (EP2_OUT_Callback ISR)
 * and delivering them to `usb_cdc_task`. Capacity: 256 bytes.
 */
extern QueueHandle_t cdc_rx_queue;

/**
 * @brief Initializes the USB hardware peripheral, descriptors, power registers, and interrupt controllers.
 */
void usb_cdc_task_init(void);

/**
 * @brief FreeRTOS Task (Priority 4, Stack 512 words) handling USB Virtual COM Port communication.
 * Parses CLI and JSON commands ("DOWNLOAD", "get-reading", "get-config", "set-config", "reset").
 * @param pvParameters FreeRTOS task creation parameter (unused).
 */
void usb_cdc_task(void *pvParameters);

#endif /* __USB_CDC_TASK_H */

