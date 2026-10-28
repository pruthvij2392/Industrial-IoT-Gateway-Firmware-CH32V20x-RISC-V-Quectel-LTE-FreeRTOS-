#ifndef __IOT_TASK_H
#define __IOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Main IoT Supervisor Task (FreeRTOS Task 1, Priority 5).
 * Orchestrates Modbus RTU polling, Quectel cellular/MQTT engine,
 * 1-second RTC tick processing, and offline flash archiving.
 * @param pvParameters FreeRTOS task creation parameter (unused).
 */
void iot_task(void *pvParameters);

#endif /* __IOT_TASK_H */

