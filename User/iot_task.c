#include "iot_task.h"
#include "comman.h"
#include "ds3231.h"
#include "quectel_ioc.h"
#include "rs485.h"
#include "w25qxx.h"

void One_Sec_Callback(void);

/**
 * @brief Primary FreeRTOS Application Task (Priority 5, Stack 512 words).
 * 
 * Execution Flow:
 * 1. Initializes RTC interrupt (DS3231 EXTI0 1Hz SQW).
 * 2. Reads non-volatile system configuration and power-fail pointers from SPI Flash.
 * 3. Initializes RS-485 USART1 bus with configured baud rate, parity, and stop bits.
 * 4. Powers on Quectel 4G modem via hardware pin sequencing (IOT_ON).
 * 5. Pre-computes Modbus register mapping pointers for quick lookup.
 * 6. Enters main infinite control loop:
 *    - Detects 1-second ticks from DS3231 SQW or software fallback timer.
 *    - Steps the Quectel AT / MQTT state machine.
 *    - Processes incoming JSON commands from MQTT topic.
 *    - Sequentially polls Modbus RTU slaves in a round-robin schedule.
 *    - Parses received Modbus RTU response packets.
 */
void iot_task(void *pvParameters) {
  uint32_t sqw_timeout_cnt = 0; // Software fallback counter in case RTC SQW pin is disconnected

  /* 1. Initialize RTC 1Hz interrupt line */
  DS3231_INT_Init();
  vTaskDelay(pdMS_TO_TICKS(100));

  /* 2. Load stored system configuration & power-fail indices from external SPI Flash */
  Set_mem_Read();
  PF_mem_Read();
  vTaskDelay(pdMS_TO_TICKS(100));

  /* 3. Initialize RS-485 USART1 interface based on loaded configuration */
  RS485_Init();
  vTaskDelay(pdMS_TO_TICKS(50));

  /* 4. Restore message transmission sequence count from power-fail memory */
  temp.SendCnt = PF.Massege_No;

  /* 5. Power ON Quectel cellular modem hardware */
  IOT_ON();

  /* 6. Pre-calculate pointer addresses in the Modbus holding register table */
  for (uint8_t fd = 0; fd < number_of_slave; fd++) {
    uint8_t row = (uint8_t)(first.DisValAdd[fd] / 100);
    uint8_t col = (uint8_t)(first.DisValAdd[fd] % 100);
    modbus.HoldAdd[fd] = &modbus.HoldReg[row][col];
  }

  /* 7. Initialize Modbus sensor string arrays to "0" */
  memset(Run.Modbus_Val, 0, sizeof(Run.Modbus_Val));
  memset(Send.Modbus_Val, 0, sizeof(Send.Modbus_Val));
  for (int i = 0; i < 16; i++) {
    Run.Modbus_Val[i][0] = '0';
    Send.Modbus_Val[i][0] = '0';
  }

  /* Start Modbus polling cycle in Mode 1 (ready to transmit request) */
  modbus.Mode = 1;

  /* ============================================================================
   * MAIN APPLICATION CONTROL LOOP
   * ============================================================================ */
  while (1) {
    /* Check for 1-second pulse from DS3231 hardware interrupt */
    if (temp.g_sqw_flag) {
      temp.g_sqw_flag = 0;
      sqw_timeout_cnt = 0;
      One_Sec_Callback();
    } else {
      /* Fallback watchdog: if hardware SQW pulse is missing for 200 iterations (~2 sec), fire callback */
      sqw_timeout_cnt++;
      if (sqw_timeout_cnt >= 200) {
        sqw_timeout_cnt = 0;
        One_Sec_Callback();
      }
    }

    /* Drive Quectel Cellular Modem & MQTT communication state machine */
    GSM_TO_MQTT_FUNC();

    /* If modem is idle (Mode 70) and a sample is ready to send, transition to publish state (Mode 21) */
    if (GSM.MODE == 70 && GSM.F_Send_Sample == 1) {
      GSM.F_Send_Sample = 0;
      GSM.MODE = 21;
    }

    /* Process pending JSON command received over MQTT */
    if (GSM.json_ready_flag) {
      GSM.json_ready_flag = 0;

      /* Safely parse JSON buffer under mutex protection */
      if (json_mutex != NULL &&
          xSemaphoreTake(json_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Parse_Custom_JSON((char *)GSM.CMD_Buff);
        xSemaphoreGive(json_mutex);
      }
    }

    /* Modbus RTU Polling Engine: Mode 1 = Transmit request to next slave */
    if (modbus.Mode == 1) {
      Modbus_Send(modbus.Device_Cnt); // Send request to active slave
      modbus.Device_Cnt++;
      if (modbus.Device_Cnt >= first.No_Of_Slave) {
        modbus.Device_Cnt = 0; // Wrap round-robin counter back to first slave
      }
      modbus.Tm_count = 0;
      modbus.Mode = 2; // Switch to wait-for-response mode
    }

    /* Check if full Modbus RTU response frame has been received by USART1 ISR */
    if (modbus.F_Rx_CHECK) {
      Modbus_Data_Rx(); // Validate CRC and extract registers
      modbus.F_Rx_CHECK = 0;
    }

    /* Yield task for 10ms (FreeRTOS scheduler allows lower priority tasks to run) */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Periodic 1-Second Heartbeat Callback.
 * Executed once per second (triggered by DS3231 1Hz SQW pulse or fallback timer).
 * - Reads current time from DS3231 RTC into the `Run` telemetry structure.
 * - Increments sampling timer (`GSM.Sample_TM_CNT`).
 * - On sample timeout, stages telemetry snapshot (`Send = Run`), logs to SPI Flash,
 *   and triggers MQTT cloud transmission (`GSM.F_Send_Sample = 1`).
 * - Manages idle-state (Mode 70) transitions for config replies, broker switches,
 *   and signal quality checks (CSQ).
 * - Restarts Modbus polling round if currently in wait state.
 */
void One_Sec_Callback(void) {
  /* Read fresh timestamp from DS3231 RTC */
  ds3231_get_time(&Run);

  /* Recovery check: if time is uninitialized (all zeros), re-init RTC interrupt and re-read */
  if (Run.year == 0 && Run.month == 0 && Run.date == 0 && Run.hour == 0 &&
      Run.minute == 0 && Run.second == 0) {
    DS3231_INT_Init();
    ds3231_get_time(&Run);
  }

  /* Advance telemetry sampling counter */
  ++GSM.Sample_TM_CNT;
  if (GSM.Sample_TM_CNT >= first.Sample_Time) {
    GSM.Sample_TM_CNT = 0; // Reset sample timer

    Send = Run; // Capture atomic snapshot of sensor readings and time for transmission

    /* Log frame to external SPI Flash ring buffer unless forced live query */
    if (GSM.F_Send_Sample != 2) {
      FRAME_WRITE_TO_ExFLASH();
    }

    GSM.F_Send_Sample = 1; // Flag that new telemetry data is ready to publish
    temp.F_Power_On = 1;   // Mark initial sample completed
  } else if (GSM.F_Send_Sample == 2) {
    /* Handle immediate on-demand reading request (from get-reading command) */
    Send = Run;
    GSM.F_Send_Sample = 1;
  }

  /* If modem is in idle state (Mode 70), determine next action */
  if (GSM.MODE == 70) {
    /* 1. Check for pending configuration response to publish */
    if (GSM.M_SendConfig == 1 || GSM.M_SendConfig == 10 ||
        GSM.M_SendConfig == 20) {
      if (GSM.M_SendConfig == 1) {
        GSM.M_SendConfig = 2;   // Staged for config payload publish
      } else if (GSM.M_SendConfig == 10) {
        GSM.M_SendConfig = 11;  // Staged for success reply publish
      } else if (GSM.M_SendConfig == 20) {
        GSM.M_SendConfig = 21;  // Staged for error reply publish
      }
      GSM.F_ResponseCheck = 0;
      GSM.MODE = 21;            // Jump to MQTT publish state
    }
    /* 2. Check for pending MQTT broker migration request */
    else if (GSM.F_Server_Change == 2) {
      GSM.F_Server_Change = 3;
      GSM.MODE = 34;            // Jump to MQTT disconnect & reconnect state
    }
    /* 3. Check for pending periodic telemetry transmission */
    else if (GSM.F_Send_Sample == 1) {
      GSM.F_Send_Sample = 0;
      GSM.MODE = 21;            // Jump to MQTT publish state
    }
    /* 4. Otherwise, perform background cellular signal quality check */
    else {
      GSM.MODE = 41;            // Jump to AT+CSQ signal strength query state
    }
  }

  /* Reset Modbus state machine to initiate next query */
  if (modbus.Mode == 2) {
    modbus.Mode = 1;
  }
}