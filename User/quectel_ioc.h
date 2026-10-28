#ifndef __QUECTEL_IOC_H
#define __QUECTEL_IOC_H

#include "ch32v20x.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "comman.h"

/**
 * @brief Main state machine and buffer structure for Quectel Cellular Modem & MQTT operations.
 */
typedef struct {
    uint16_t Time_count;        /**< Timeout loop counter for awaiting AT command responses */
    uint16_t Sample_TM_CNT;     /**< Telemetry sampling interval countdown counter */
    uint16_t ResponseCheckTM;   /**< Max timeout ticks allowed for specific AT command response */
    uint16_t Wr_Len;            /**< Number of bytes in transmission buffer */
    uint16_t MQTTFrameLen;      /**< Length of active outgoing MQTT JSON payload string */
    uint16_t RdCheckLen;        /**< Length of expected response string in RdCheckBuff */
    uint16_t Rd_Cnt;            /**< Current write index / byte count in Rd_Buff */
    uint16_t Rd_Cnt2;           /**< Current write index for CMD_Buff (incoming JSON) */
    uint16_t gsm_pwr_cnt;       /**< Power sequencing delay counter */
    uint8_t Wr_Buff[512];       /**< Outgoing AT command formatting buffer */
    uint8_t MQTTFrame[512];     /**< Staged outgoing MQTT JSON frame buffer (with trailing \x1A) */
    uint8_t Rd_Buff[512];       /**< Incoming UART response buffer from Quectel modem */
    uint8_t RdCheckBuff[64];    /**< Target substring expected in modem response (e.g. "OK", "+QMTOPEN:") */
    uint8_t NW_Status;          /**< 0: Disconnected, 1: Connected/Subscribed, 2: Connecting */
    uint8_t M_SendConfig;       /**< Config response state (1: get-config, 10: success, 20: error) */
    uint8_t M_BrokerChange;     /**< Broker change sequencing step */
    uint8_t F_Server_Change;    /**< Flag indicating MQTT broker migration requested */
    uint8_t F_Error;            /**< Modem general error flag */
    uint8_t Signal_Val;         /**< Raw CSQ RSSI value (0 to 31, or 99 for unknown) */
    uint8_t MODE;               /**< Active state machine index in GSM_TO_MQTT_FUNC() (0 to 73) */
    uint8_t Error_cnt;          /**< Consecutive AT command retry / error counter */
    uint8_t F_NW;               /**< 1 when registered to cellular network (CEREG 1 or 5) */
    uint8_t F_Send_Sample;      /**< 1: Telemetry sample ready to publish, 2: Immediate sample request */
    uint8_t F_ResponseCheck;    /**< 1: Waiting, 2: Matched expected response, 3: Timed out / error */
    uint8_t CMD_Buff[512];      /**< Buffer holding incoming JSON command received from MQTT topic */
    int8_t brace_count;         /**< Curly brace counter ('{', '}') to detect complete balanced JSON */
    char Config_str[512];       /**< Formatted configuration string for get-config response */
    bool SimNo;                 /**< Active SIM slot (0: SIM Slot 1, 1: SIM Slot 2) */
    bool SimSwitch;             /**< Flag requesting SIM slot switch */
    bool F_NewFrame;            /**< Set when opening '{' is detected in UART RX stream */
    bool json_ready_flag;       /**< Set when complete balanced JSON frame is ready for parsing */
    bool GPS_FIX;               /**< 1 if GNSS has valid satellite position lock ('A'); 0 if void ('V') */
} GSM_Struct;

extern GSM_Struct GSM;
extern SemaphoreHandle_t json_mutex;

extern char mqtt_version[6];
extern char error_msg[128];
extern char API_action[16];
extern uint16_t error_code;

/**
 * @brief Powers on Quectel modem via PB4 (POWER) and PB3 (PWRKEY) pin sequence.
 */
void IOT_ON(void);

/**
 * @brief Primary Quectel AT Command & MQTT state machine engine. Called in iot_task loop.
 */
void GSM_TO_MQTT_FUNC(void);

/**
 * @brief Parses JSON frames received over MQTT topic and executes configuration / action commands.
 * @param json_frame Pointer to null-terminated JSON command string.
 */
void Parse_Custom_JSON(const char* json_frame);
void ProcessGPSFrame();
void Quectel_Init(void);

/**
 * @brief Updates LED blink and solid patterns for NW_LED, GPS_LED, and DATA_LED based on modem status.
 */
void GSM_LED_Handler(void);

#endif /* __QUECTEL_IOC_H */

