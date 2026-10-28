#include "usb_cdc_task.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "comman.h"
#include "task.h"
#include "w25qxx.h"
#include "quectel_ioc.h"
#include <stdio.h>
#include <string.h>

/* Queue handle for holding received USB characters from the USB ISR */
QueueHandle_t cdc_rx_queue = NULL;

/* Private snapshot structure for USB telemetry queries */
static Run_Send_t cdc_send = {0};

/* External USB driver symbols from WCH USBLIB */
extern uint8_t USBD_ENDPx_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len);
extern volatile uint8_t USBD_Endp3_Busy;

/**
 * @brief Transmits a buffer of arbitrary length to the USB Host over CDC Endpoint 3.
 * 
 * Flow & Robustness Details:
 * 1. Chunks data into maximum 64-byte packets (the USB Full-Speed max packet size for bulk endpoints).
 * 2. Checks hardware endpoint TX status: if host already fetched previous packet, clears busy flag.
 * 3. Implements a 500ms timeout (250 ticks x 2ms tick period). If host cable is unplugged
 *    or host application stops reading, gracefully aborts and resets busy flag to prevent permanent task deadlock.
 * 4. Calls USBD_ENDPx_DataUp inside taskENTER_CRITICAL() / taskEXIT_CRITICAL() to prevent USB ISR race conditions.
 * 5. Waits for the final packet of the message to be collected by host.
 * 
 * @param pbuf Pointer to data bytes to transmit.
 * @param len Total number of bytes to send.
 * @return true if all bytes were successfully sent to host; false if host timed out or disconnected.
 */
static bool CDC_Transmit(const uint8_t *pbuf, uint16_t len) {
  uint16_t offset = 0;
  while (len > 0) {
    uint16_t chunk = (len > 64) ? 64 : len;

    // Check hardware status: if endpoint is not actively transmitting, clear busy flag
    if (GetEPTxStatus(ENDP3) != EP_TX_VALID) {
      USBD_Endp3_Busy = 0;
    }

    // Wait until Endpoint 3 is ready (not busy)
    uint32_t wait_ticks = 250; // 250 ticks * 2ms = 500ms timeout
    while (USBD_Endp3_Busy && wait_ticks > 0) {
      vTaskDelay(1); // 1 tick = 2ms allows USB interrupt to fire and other tasks to run
      if (GetEPTxStatus(ENDP3) != EP_TX_VALID) {
        USBD_Endp3_Busy = 0;
        break;
      }
      wait_ticks--;
    }
    if (USBD_Endp3_Busy) {
      // Force recover flag to avoid permanent lockout on subsequent calls
      USBD_Endp3_Busy = 0;
      return false; // Host disconnected or stopped reading
    }

    taskENTER_CRITICAL();
    uint8_t res = USBD_ENDPx_DataUp(ENDP3, (uint8_t *)&pbuf[offset], chunk);
    taskEXIT_CRITICAL();

    if (res != USB_SUCCESS) {
      USBD_Endp3_Busy = 0;
      return false;
    }

    len -= chunk;
    offset += chunk;
  }

  // Wait until the final packet of this line has been transferred to host
  uint32_t final_ticks = 250;
  while (USBD_Endp3_Busy && final_ticks > 0) {
    vTaskDelay(1);
    if (GetEPTxStatus(ENDP3) != EP_TX_VALID) {
      USBD_Endp3_Busy = 0;
      break;
    }
    final_ticks--;
  }

  // Always ensure flag is clean so next transmission is never blocked
  USBD_Endp3_Busy = 0;
  return true;
}

/**
 * @brief Streams the entire historical data archive from SPI Flash over USB CDC as a CSV file.
 * Triggered by receiving the plain text command "DOWNLOAD".
 * 
 * Execution Steps:
 * 1. Transmits CSV Header row:
 *    "Sr No,DATE,TIME,LATITUDE,LONGITUDE,v1..v16,SIGNAL STRENGTH\r\n".
 * 2. Iterates over all valid stored records (`Total_Records = PF.Rd_Frame_idx`).
 * 3. Yields CPU every 16 records (`vTaskDelay(1)`) so other system tasks can run.
 * 4. Reads each 256-byte frame from SPI Flash under `flash_mutex` protection.
 * 5. Sanitizes sensor ASCII strings (ensures printable characters 32-126).
 * 6. Formats record into a CSV row and transmits over CDC.
 * 7. Aborts immediately if host stops reading or disconnects.
 */
static void Send_All_Frames_USB(void) {
  uint32_t i_eep, eep_add = 0, Total_Records = 0;
  bool preDone = 0;
  uint16_t frameNo = 0;
  static char g_write_buffer[512];

  // Send CSV Header
  const char *header_text =
      "Sr No,DATE,TIME,LATITUDE,LONGITUDE,"
      "v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15,v16,"
      "SIGNAL STRENGTH\r\n";

  if (!CDC_Transmit((const uint8_t *)header_text, strlen(header_text))) {
    return;
  }

  eep_add = 0;
  frameNo = 0;
  preDone = 0;
  Total_Records = PF.Rd_Frame_idx;

  for (i_eep = 0; i_eep < Total_Records; i_eep++) {
    if ((i_eep % 16) == 0) {
      vTaskDelay(1); // Yield to other tasks (1 tick = 2ms)
    }

    /* Handle memory wrap-around pointer calculations */
    if (PF.FlashFull == 1) {
      if ((eep_add == (PF.flash_Add + DataFrameByte)) &&
          (PF.Massege_No != 1)) {
        if (PF.frame_idx != (4096 / DataFrameByte)) {
          eep_add =
              (uint32_t)((LastSect * 4096) + (PF.frame_idx * DataFrameByte));
          preDone = 1;
        }
      }
    }

    Run_Send_t sd_send = {0};
    if (flash_mutex != NULL) {
      xSemaphoreTake(flash_mutex, portMAX_DELAY); // Thread-safe flash read
    }
    SPI_Flash_Read((uint8_t *)&sd_send, eep_add, sizeof(sd_send));
    if (flash_mutex != NULL) {
      xSemaphoreGive(flash_mutex);
    }

    /* Convert floating point coordinates to integer and micro-degree parts */
    int lat_int = (int)sd_send.latitude;
    int lat_dec = (int)((sd_send.latitude - lat_int) * 1000000.0f);
    if (lat_dec < 0)
      lat_dec = -lat_dec;

    int lon_int = (int)sd_send.longitude;
    int lon_dec = (int)((sd_send.longitude - lon_int) * 1000000.0f);
    if (lon_dec < 0)
      lon_dec = -lon_dec;

    /* Validate Modbus sensor string values */
    const char *v[16];
    for (int j = 0; j < 16; j++) {
      bool valid = false;
      for (int k = 0; k < 15; k++) {
        char c = sd_send.Modbus_Val[j][k];
        if (c == '\0')
          break;
        if (c >= 32 && c <= 126)
          valid = true;
        else {
          valid = false;
          break;
        }
      }
      v[j] = (valid && sd_send.Modbus_Val[j][0] != '\0')
                 ? sd_send.Modbus_Val[j]
                 : "0";
    }

    /* Format CSV record */
    int len = snprintf(
        g_write_buffer, sizeof(g_write_buffer),
        "%lu,20%02u-%02u-%02u,%02u:%02u:%02u,%d.%06d,%d.%06d,"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u\r\n",
        (unsigned long)(i_eep + 1), sd_send.year, sd_send.month, sd_send.date,
        sd_send.hour, sd_send.minute, sd_send.second, lat_int, lat_dec,
        lon_int, lon_dec, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
        v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15],
        sd_send.Signal_Strength);

    if (!CDC_Transmit((const uint8_t *)g_write_buffer, len)) {
      break; // Stop streaming if USB host disconnects or stops reading
    }

    eep_add += DataFrameByte;
    frameNo++;
    if (frameNo >= (4096 / DataFrameByte)) {
      frameNo = 0;
      if (PF.FlashFull == 1) {
        if (preDone) {
          preDone = 0;
          eep_add = (uint32_t)((PF.sector + 1) * 4096);
        }
      }
    }
  }
}

/**
 * @brief Initializes USB CDC hardware, endpoint queues, and interrupt configurations.
 */
void usb_cdc_task_init(void) {
  // Create FreeRTOS queue for CDC RX data (256 bytes)
  cdc_rx_queue = xQueueCreate(256, sizeof(uint8_t));

  // Initialize USB hardware and clocks
  Set_USBConfig();
  USB_Init();
  USB_Interrupts_Config();
}

/**
 * @brief FreeRTOS Task (Priority 4, Stack 512 words) for USB Virtual COM Port communication.
 * 
 * Command Handling:
 * 1. Accumulates characters until newline ('\r' or '\n').
 * 2. If command is "DOWNLOAD": triggers full historical CSV data dump via Send_All_Frames_USB().
 * 3. If command is JSON:
 *    - "act":"get-reading": returns live telemetry reading in JSON.
 *    - "act":"get-config": returns complete device configuration in 2-chunk JSON.
 *    - "act":"set-config": parses and applies configuration updates using Parse_Custom_JSON().
 *      (Carefully saves and restores Quectel MQTT state flags so USB configuration does not interrupt cloud publishes).
 *    - "act":"reset": resets flash pointers and message counters.
 *    - Handles version error (400), duplicate data (401), invalid range (403), and command mismatch (405).
 */
void usb_cdc_task(void *pvParameters) {
  uint8_t rx_byte;
  static uint8_t rx_buffer[128];
  static uint8_t tx_buffer[1024];
  uint16_t rx_idx = 0;

  usb_cdc_task_init();

  while (1) {
    /* Receive next byte from USB RX queue with 50-tick (100ms) timeout */
    if (xQueueReceive(cdc_rx_queue, &rx_byte, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (rx_byte == '\n' || rx_byte == '\r') {
        if (rx_idx == 0) {
          // Ignore leading or consecutive CR/LF
          continue;
        }
        rx_buffer[rx_idx] = '\0'; // Null-terminate command string

        // Check for CSV download command (plain text "DOWNLOAD")
        if (strstr((char *)rx_buffer, "DOWNLOAD") != NULL) {
          Send_All_Frames_USB();
          rx_idx = 0;
          continue;
        }

        memset(tx_buffer, 0, sizeof(tx_buffer));

        /* Process JSON commands */
        if (rx_idx > 5 && rx_buffer[0] == '{') {
          /* 1. Command: "get-reading" */
          if (strstr((char *)rx_buffer, "\"act\":\"get-reading\"") != NULL) {
            char temp_str[41];
            sprintf(temp_str, "\"ver\":\"%s\"", mqtt_version);
            if (strstr((char *)rx_buffer, temp_str) == NULL) {
              sprintf(
                  (char *)tx_buffer,
                  "{\"st\":\"error\",\"ver\":\"%s\",\"act\":\"get-reading\","
                  "\"err\":{\"ec\":400,\"em\":\"version error\"}}\r\n",
                  mqtt_version);
            } else {
              taskENTER_CRITICAL();
              cdc_send = Run; // Atomic copy of live telemetry
              taskEXIT_CRITICAL();

              int lat_int = (int)cdc_send.latitude;
              int lat_dec = (int)((cdc_send.latitude - lat_int) * 1000000.0f);
              if (lat_dec < 0)
                lat_dec = -lat_dec;

              int lon_int = (int)cdc_send.longitude;
              int lon_dec = (int)((cdc_send.longitude - lon_int) * 1000000.0f);
              if (lon_dec < 0)
                lon_dec = -lon_dec;

              const char *v[16];
              for (int i = 0; i < 16; i++) {
                v[i] = (cdc_send.Modbus_Val[i][0] != '\0')
                           ? cdc_send.Modbus_Val[i]
                           : "0";
              }

              snprintf((char *)tx_buffer, sizeof(tx_buffer),
                       "{\"ts\":\"20%02u-%02u-%02uT%02u:%02u:%02u\","
                       "\"la\":%d.%06d," // latitude
                       "\"lo\":%d.%06d," // longitude
                       "\"xg\":%u,"      // signal strength
                       "\"d\":"
                       "{"
                       "\"v1\":%s,"
                       "\"v2\":%s,"
                       "\"v3\":%s,"
                       "\"v4\":%s,"
                       "\"v5\":%s,"
                       "\"v6\":%s,"
                       "\"v7\":%s,"
                       "\"v8\":%s,"
                       "\"v9\":%s,"
                       "\"v10\":%s,"
                       "\"v11\":%s,"
                       "\"v12\":%s,"
                       "\"v13\":%s,"
                       "\"v14\":%s,"
                       "\"v15\":%s,"
                       "\"v16\":%s"
                       "}"
                       "}\r\n",
                       cdc_send.year, cdc_send.month, cdc_send.date,
                       cdc_send.hour, cdc_send.minute, cdc_send.second, lat_int,
                       lat_dec, lon_int, lon_dec, cdc_send.Signal_Strength,
                       v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8],
                       v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
            }
          }
          /* 2. Command: "get-config" */
          else if (strstr((char *)rx_buffer, "\"act\":\"get-config\"") != NULL) {
            char temp_str[41];
            sprintf(temp_str, "\"ver\":\"%s\"", mqtt_version);
            if (strstr((char *)rx_buffer, temp_str) == NULL) {
              sprintf(
                  (char *)tx_buffer,
                  "{\"st\":\"error\",\"ver\":\"%s\",\"act\":\"get-config\","
                  "\"err\":{\"ec\":400,\"em\":\"version error\"}}\r\n",
                  mqtt_version);
            } else {
              int len = snprintf(
                  (char *)tx_buffer, sizeof(tx_buffer),
                  "{\"st\":\"success\",\"ver\":\"%s\",\"act\":\"get-config\","
                  "\"d\":{\"sn\":\"GTW%s\",\"dn\":\"%s\",\"bip\":\"%s\","
                  "\"bun\":\"%s\",\"bpt\":%u,\"spt\":%u,\"bdr\":%u,"
                  "\"prt\":%u,\"spb\":%u,\"tts\":%u",
                  mqtt_version, first.Serial_no, first.Device_ID,
                  first.MQTTServer, first.MQTTUsername,
                  (unsigned int)first.MQTTPort, (unsigned int)first.Sample_Time,
                  (unsigned int)first.Baud_Rate, (unsigned int)first.Parity,
                  (unsigned int)first.Stop_bit, (unsigned int)first.No_Of_Slave);

              uint8_t count = (first.No_Of_Slave > 16) ? 16 : first.No_Of_Slave;
              for (uint8_t i = 0; i < count; i++) {
                if (len < (int)sizeof(tx_buffer)) {
                  len += snprintf(
                      (char *)tx_buffer + len, sizeof(tx_buffer) - len,
                      ",\"s%u\":{\"sid\":%u,\"fc\":%u,\"ad\":%u,\"qnt\":%u}",
                      (unsigned int)(i),
                      (unsigned int)first.SlaveID[i],
                      (unsigned int)first.FunctionID[i],
                      (unsigned int)first.ModbusAdd[i],
                      (unsigned int)first.BytetoRead[i]);
                }
              }

              // Transmit base config + s frames first (Chunk 1)
              if (CDC_Transmit((const uint8_t *)tx_buffer, len)) {
                // Prepare v1 to v16 frames + closing }}\r\n (Chunk 2)
                len = 0;
                for (uint8_t k = 0; k < 16; k++) {
                  uint8_t row = (uint8_t)(first.DisValAdd[k] / 100);
                  uint8_t col = (uint8_t)(first.DisValAdd[k] % 100);

                  if (len < (int)sizeof(tx_buffer)) {
                    len += snprintf(
                        (char *)tx_buffer + len, sizeof(tx_buffer) - len,
                        ",\"v%u\":{\"vid\":%u,\"reg\":%u,\"dtp\":%u,\"edn\":%u,\"dp\":%u}",
                        (unsigned int)(k),
                        (unsigned int)first.SlaveID[row],
                        (unsigned int)(col / 2),
                        (unsigned int)first.DataType[k],
                        (unsigned int)first.Endian[k],
                        (unsigned int)first.Decimal[k]);
                  }
                }

                if (len < (int)sizeof(tx_buffer)) {
                  len += snprintf((char *)tx_buffer + len, sizeof(tx_buffer) - len,
                                  "}}\r\n");
                }

                // Transmit v frames and closing braces (Chunk 2)
                CDC_Transmit((const uint8_t *)tx_buffer, len);
              }

              // Clear tx_buffer so outer transmission block is not triggered
              tx_buffer[0] = '\0';
            }
          }
          /* 3. Command: "set-config" or "reset" */
          else {
            if (json_mutex != NULL &&
                xSemaphoreTake(json_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
              /* Save current GSM state flags to avoid disrupting cloud MQTT session */
              uint8_t saved_send_config = GSM.M_SendConfig;
              uint8_t saved_send_sample = GSM.F_Send_Sample;

              GSM.M_SendConfig = 0;
              Parse_Custom_JSON((char *)rx_buffer);

              if (GSM.M_SendConfig == 10) {
                sprintf(
                    (char *)tx_buffer,
                    "{\"st\":\"success\",\"ver\":\"%s\",\"act\":\"%s\"}\r\n",
                    mqtt_version, API_action);
              } else if (GSM.M_SendConfig == 20) {
                sprintf((char *)tx_buffer,
                        "{\"st\":\"error\",\"ver\":\"%s\",\"act\":\"%s\","
                        "\"err\":{\"ec\":%u,\"em\":\"%s\"}}\r\n",
                        mqtt_version, API_action, error_code, error_msg);
              } else {
                sprintf((char *)tx_buffer,
                        "{\"st\":\"error\",\"ver\":\"%s\",\"act\":\"unknown\","
                        "\"err\":{\"ec\":405,\"em\":\"command not match\"}}\r\n",
                        mqtt_version);
              }

              // Restore GSM state so USB CDC does not disrupt Quectel MQTT publish
              GSM.M_SendConfig = saved_send_config;
              GSM.F_Send_Sample = saved_send_sample;

              xSemaphoreGive(json_mutex);
            }
          }
        } else if (rx_idx > 1) {
          sprintf((char *)tx_buffer,
                  "{\"st\":\"error\",\"ver\":\"%s\",\"act\":\"unknown\","
                  "\"err\":{\"ec\":405,\"em\":\"command not match\"}}\r\n",
                  mqtt_version);
        }

        /* Transmit response buffer if populated */
        if (tx_buffer[0] != '\0') {
          CDC_Transmit((const uint8_t *)tx_buffer, strlen((char *)tx_buffer));
        }

        rx_idx = 0; // Reset receive index for next command
      } else {
        /* Append byte to incoming buffer */
        if (rx_idx < sizeof(rx_buffer) - 1) {
          rx_buffer[rx_idx++] = rx_byte;
        } else {
          rx_idx = 0; // Discard on overflow
        }
      }
    }
  }
}
