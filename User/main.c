/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : Main program entry point and FreeRTOS task orchestration.
 * Microcontroller    : WCH CH32V203 (RISC-V QingKe V4F Core)
 *******************************************************************************/

#include "FreeRTOS.h"
#include "comman.h"
#include "debug.h"
#include "ds3231.h"
#include "iot_task.h"
#include "quectel_ioc.h"
#include "rs485.h"
#include "task.h"
#include "usb_cdc_task.h"
#include "w25qxx.h"

/* ==============================================================================
 * FREERTOS TASK CONFIGURATION CONSTANTS
 * ============================================================================== */

/**
 * @brief Task 1 (iot_task) Configuration:
 * - Priority: 5 (Higher priority, guarantees strict Modbus RTU timing and cellular responsiveness).
 * - Stack Size: 512 words (512 * 4 bytes = 2048 bytes).
 */
#define TASK1_TASK_PRIO 5
#define TASK1_STK_SIZE 512

/**
 * @brief Task 2 (usb_cdc_task) Configuration:
 * - Priority: 4 (Lower priority than IoT task to ensure field telemetry is never blocked by USB).
 * - Stack Size: 512 words (512 * 4 bytes = 2048 bytes).
 */
#define TASK2_TASK_PRIO 4
#define TASK2_STK_SIZE 512

/* FreeRTOS Task Handles */
TaskHandle_t IotTask_Handler;    /**< Handle for managing and inspecting iot_task */
TaskHandle_t UsbCdcTask_Handler; /**< Handle for managing and inspecting usb_cdc_task */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main hardware startup and FreeRTOS scheduler entry point.
 *
 * Hardware Startup Sequence:
 * 1. Configure NVIC interrupt priority grouping (Group 1: 1 preemption bit, 3 subpriority bits).
 * 2. Update SystemCoreClock variable to match current PLL frequency (e.g. 144MHz or 72MHz).
 * 3. Initialize low-level delay timers.
 * 4. Initialize board peripherals:
 *    - GPIO pins for LEDs and modem power control.
 *    - USART2 for Quectel 4G LTE cellular modem.
 *    - SPI1 for external Winbond Flash (W25Q series).
 *    - I2C1 for Maxim DS3231 Real-Time Clock.
 * 5. Remap JTAG pins to free PB3/PB4 for modem control while retaining SWD debug.
 * 6. Create FreeRTOS tasks and hand over execution to the kernel scheduler.
 *
 * @return  int (never returns under normal FreeRTOS operation)
 *********************************************************************/
int main(void) {
  /* 1. Configure Interrupt Priority Grouping */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* 2. Synchronize CPU clock speed variable */
  SystemCoreClockUpdate();

  /* 3. Initialize SysTick-based microsecond delay helper */
  Delay_Init();

  /* 4. Initialize on-board hardware peripherals */
  gpio_pin_init();  // Configure LED pins (PA1, PA15, PB0, PB1) and modem pins (PB3, PB4)
  Quectel_Init();   // Configure USART2 (PA2-TX, PA3-RX at 115200 baud) and interrupt for 4G modem
  W25QXX_Init();    // Configure SPI1 (PA4-CS, PA5-SCK, PA6-MISO, PA7-MOSI) for flash memory
  DS3231_Init();    // Configure I2C1 (PB6-SCL, PB7-SDA at 100kHz) for external RTC

  /* 5. Remap JTAG pins to release PB3 and PB4 for general-purpose I/O.
   * By default, PB3 is JTDO and PB4 is JNTRST. Disabling JTAG frees these pins
   * so they can control the Quectel Modem's PWRKEY and POWER pins,
   * while keeping SWD (PA13/PA14) active for flashing and debugging.
   */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
#ifndef GPIO_Remap_SWJ_JTAGDisable
#define GPIO_Remap_SWJ_JTAGDisable ((uint32_t)0x00300200)
#endif
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  /* 6. Create FreeRTOS Tasks */

  /* Create Task 1: Main IoT Master Supervisor */
  xTaskCreate((TaskFunction_t)iot_task, (const char *)"iot_task",
              (uint16_t)TASK1_STK_SIZE, (void *)NULL,
              (UBaseType_t)TASK1_TASK_PRIO, (TaskHandle_t *)&IotTask_Handler);

  /* Create Task 2: USB Virtual COM Port (CDC) Interface */
  xTaskCreate((TaskFunction_t)usb_cdc_task, (const char *)"usb_cdc_task",
              (uint16_t)TASK2_STK_SIZE, (void *)NULL,
              (UBaseType_t)TASK2_TASK_PRIO,
              (TaskHandle_t *)&UsbCdcTask_Handler);

  /* 7. Start the FreeRTOS Preemptive Multitasking Scheduler */
  vTaskStartScheduler();

  /* The following loop is only reached if FreeRTOS heap memory is insufficient to start the scheduler */
  while (1) {
    printf("shouldn't run at here!!\n");
  }
}
