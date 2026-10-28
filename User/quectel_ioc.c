#include "quectel_ioc.h"
#include "FreeRTOS.h"
#include "comman.h"
#include "ds3231.h"
#include "rs485.h"
#include "task.h"
#include "w25qxx.h"

/* ==============================================================================
 * GLOBAL & STATIC QUECTEL SUBSYSTEM VARIABLES
 * ============================================================================== */

/**
 * @brief Primary Quectel state structure holding buffers, state flags, and counters.
 */
GSM_Struct GSM = {0};

/**
 * @brief Telemetry frame snapshot staged for cellular MQTT transmission.
 */
static Run_Send_t iot_send = {0};

/**
 * @brief Active communication protocol version. All JSON requests must match this.
 */
char mqtt_version[6] = "2.0.0";
char error_msg[128];
char API_action[16];
uint16_t error_code;

SemaphoreHandle_t json_mutex = NULL;

/* Non-blocking UART TX Ring Buffer for sending AT commands to Quectel Modem */
#define QUECTEL_TX_BUF_SIZE 512
static uint8_t quectel_tx_buf[QUECTEL_TX_BUF_SIZE];
static volatile uint16_t quectel_tx_head = 0, quectel_tx_tail = 0;

/* Prototypes */
void IOT_UART_WRITE(uint8_t *gsm_msg);
void Quectel_SendByte(uint8_t ch);

void READING_FRAME();
void CONFIG_FRAME(char *config_str);
void SUCCESS_FRAME(char *act);
void ERROR_FRAME(char *act, uint16_t ec, char *em);
void IOT_Reset(void);
void GSMResponse_Error_Handler(void);
void Wait_For_GSMResponse(void);
void GSMResponseCollect(char *CheckBuff, uint16_t CheckTm);
void GPS_DATAConversion(char *GpsStr, uint8_t CalLen, uint8_t CalOpr);

/**
 * @brief Formats the staged telemetry frame (`iot_send`) into an outgoing MQTT JSON payload.
 * 
 * Payload structure:
 * - `ts`: ISO 8601 timestamp ("20YY-MM-DDTHH:MM:SS")
 * - `la`: Latitude with 6 decimal places
 * - `lo`: Longitude with 6 decimal places
 * - `xg`: Cellular signal quality (CSQ converted to percentage, 0-100)
 * - `d`:  Sensor readings dictionary ("v1" to "v16")
 * 
 * Note: Appends ASCII 0x1A (Ctrl+Z) character at the end. In Quectel AT+QMTPUB command,
 * 0x1A signals the end of the MQTT message payload to be dispatched to the broker.
 */
void READING_FRAME() {
  int lat_int = (int)iot_send.latitude;
  int lat_dec = (int)((iot_send.latitude - lat_int) * 1000000.0f);
  if (lat_dec < 0)
    lat_dec = -lat_dec;

  int lon_int = (int)iot_send.longitude;
  int lon_dec = (int)((iot_send.longitude - lon_int) * 1000000.0f);
  if (lon_dec < 0)
    lon_dec = -lon_dec;

  const char *v[16];
  for (int i = 0; i < 16; i++) {
    v[i] = (iot_send.Modbus_Val[i][0] != '\0') ? iot_send.Modbus_Val[i] : "0";
  }

  snprintf((char *)GSM.MQTTFrame, sizeof(GSM.MQTTFrame),
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
           "}\x1A",
           iot_send.year, iot_send.month, iot_send.date, iot_send.hour,
           iot_send.minute, iot_send.second, lat_int, lat_dec, lon_int, lon_dec,
           iot_send.Signal_Strength, v[0], v[1], v[2], v[3], v[4], v[5], v[6],
           v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
  GSM.MQTTFrameLen = (uint16_t)strlen((char *)GSM.MQTTFrame);
}

/**
 * @brief Constructs a JSON response containing device serial and configuration data.
 * Appends 0x1A (Ctrl+Z) for Quectel MQTT publish completion.
 * @param config_str Substring containing formatted configuration variables.
 */
void CONFIG_FRAME(char *config_str) {
  sprintf((char *)GSM.MQTTFrame,
          "{\"st\":\"success\","    // status
          "\"ver\":\"%s\","         // version
          "\"act\":\"get-config\"," // action
          "\"d\":"
          "{"
          "\"sn\":\"GTW%s\"," // serial number
          "%s"                // config variable data
          "}"
          "}\x1A",
          mqtt_version, first.Serial_no, config_str);
  GSM.MQTTFrameLen = (uint16_t)strlen((char *)GSM.MQTTFrame);
}

/**
 * @brief Constructs a standard JSON success response packet.
 * Appends 0x1A (Ctrl+Z) for Quectel MQTT publish completion.
 * @param act Action verb that succeeded (e.g. "set-config" or "reset").
 */
void SUCCESS_FRAME(char *act) {
  sprintf((char *)GSM.MQTTFrame,
          "{\"st\":\"success\","
          "\"ver\":\"%s\","
          "\"act\":\"%s\""
          "}\x1A",
          mqtt_version, act);
  GSM.MQTTFrameLen = (uint16_t)strlen((char *)GSM.MQTTFrame);
}

/**
 * @brief Constructs a standardized JSON error response packet.
 * Appends 0x1A (Ctrl+Z) for Quectel MQTT publish completion.
 * @param act Action verb that failed.
 * @param ec Numerical error code (400, 401, 403, 405).
 * @param em Error message string.
 */
void ERROR_FRAME(char *act, uint16_t ec, char *em) {
  sprintf((char *)GSM.MQTTFrame,
          "{\"st\":\"error\","
          "\"ver\":\"%s\","
          "\"act\":\"%s\","
          "\"err\":"
          "{"
          "\"ec\":%u,"
          "\"em\":\"%s\""
          "}"
          "}\x1A",
          mqtt_version, act, ec, em);
  GSM.MQTTFrameLen = (uint16_t)strlen((char *)GSM.MQTTFrame);
}

/**
 * @brief Decodes and processes incoming JSON commands received from MQTT or USB CDC.
 * 
 * Supported Actions:
 * 1. "act":"get-config": Emits current configuration parameters.
 * 2. "act":"set-config": Updates parameters (RTC, Broker, Sample Time, Modbus map, Serial No).
 * 3. "act":"reset": Resets flash memory pointers and message sequence counters.
 * 4. "act":"get-reading": Triggers immediate sample transmission.
 * 
 * @param json_frame Pointer to null-terminated JSON command string.
 */
void Parse_Custom_JSON(const char *json_frame) {
  // float temp_float;
  int32_t temp_int;        /* Scratch variable to capture integer outputs from JSON parser */
  uint16_t password = 0;   /* Security authorization PIN (must be 9999 to update serial number) */
  char temp_str[64];       /* Scratch string buffer to capture string outputs from JSON parser */
  bool f_version = 0;      /* Flag: 0 = Version matched ("2.0.0"), 1 = Incompatible API version */
  bool f_data_update = 0;  /* Flag: 1 = One or more configuration values were altered and need flash save */

  /* Verify that the payload contains the matching API protocol version ("ver":"2.0.0") */
  sprintf(temp_str, "\"ver\":\"%s\"", mqtt_version);
  if (strstr(json_frame, temp_str) == NULL) // mqtt_version error
  {
    f_version = 1; // Flag version error if version string doesn't match
  }

  /* -------------------------------------------------------------------------
   * ACTION: "get-config"
   * Serializes current gateway system configurations into JSON string.
   * ------------------------------------------------------------------------- */
  if (strstr(json_frame, "\"act\":\"get-config\"") != NULL) {
    strcpy(API_action, "get-config");
    if (!f_version) {
      /* Pack current device parameters into GSM.Config_str:
       * "dn"  = Device Name / ID string
       * "bip" = MQTT Broker Host IP or URL
       * "bun" = MQTT Broker Username
       * "bpt" = MQTT Broker Port
       * "spt" = Sampling Cadence in seconds
       * "bdr" = RS485 Baud Rate Index (0:9600, 1:19200, 2:38400, 3:57600, 4:115200)
       * "prt" = RS485 Parity (1:None, 2:Even, 3:Odd)
       * "spb" = RS485 Stop Bits (1:1 Stop Bit, 2:2 Stop Bits)
       * "tts" = Total active Modbus sensor slaves (1 to 16)
       */
      sprintf(GSM.Config_str,
              "\"dn\":\"%s\","  // device id
              "\"bip\":\"%s\"," // mqtt broker
              "\"bun\":\"%s\"," // mqtt broker uername
              "\"bpt\":%u,"     // mqtt port
              "\"spt\":%u,"     // sampling time
              "\"bdr\":%u,"     // baudrate
              "\"prt\":%u,"     // parity
              "\"spb\":%u,"     // stopbit
              "\"tts\":%u",     // total sensor
              first.Device_ID, first.MQTTServer, first.MQTTUsername,
              first.MQTTPort, first.Sample_Time, first.Baud_Rate, first.Parity,
              first.Stop_bit, first.No_Of_Slave);
      GSM.M_SendConfig = 1; /* Stage MQTT response dispatch: mode 1 -> mode 2 -> CONFIG_FRAME */
    } else {
      error_code = 400; /* Version mismatch error */
      strcpy(error_msg, "version error");
      GSM.M_SendConfig = 20; /* Stage error response dispatch */
    }
  } else if (strstr(json_frame, "\"act\":\"set-config\"") != NULL) {
    strcpy(API_action, "set-config");
    if (!f_version) {
      error_code = 0;

      /* "ts": Timestamp synchronization ("20YY-MM-DDTHH:MM:SS" -> DS3231 RTC) */
      if (Get_JSON_String(json_frame, "\"ts\"", temp_str,
                          sizeof(temp_str))) // time stamp
      {
        RTC_WrtFromStr((uint8_t *)&temp_str[2]);
        f_data_update = 1;
      }

      /* "dn": Device Name / ID string (Max 16 characters) */
      if (Get_JSON_String(json_frame, "\"dn\"", temp_str,
                          sizeof(temp_str))) // device name
      {
        if (strlen(temp_str) == 0 || strlen(temp_str) > 16) {
          error_code = 403; /* Invalid range */
        } else if ((strcmp(first.Device_ID, temp_str)) != 0) {
          strncpy(first.Device_ID, temp_str, sizeof(first.Device_ID) - 1);
          first.Device_ID[sizeof(first.Device_ID) - 1] = '\0';
          f_data_update = 1;
        }
      }

      /* "bip": MQTT Broker Server IP or URL (Max 40 characters) */
      if (Get_JSON_String(json_frame, "\"bip\"", temp_str,
                          sizeof(temp_str))) // broker ip
      {
        if (strlen(temp_str) == 0 || strlen(temp_str) > 40) {
          error_code = 403; /* Invalid range */
        } else if ((strcmp(first.MQTTServer, temp_str)) != 0) {
          strncpy(first.MQTTServer, temp_str, sizeof(first.MQTTServer) - 1);
          first.MQTTServer[sizeof(first.MQTTServer) - 1] = '\0';
          f_data_update = 1;
          GSM.F_Server_Change = 1; /* Mark broker credentials changed to trigger reconnect */
        }
      }

      /* "bun": MQTT Broker Username (Max 40 characters) */
      if (Get_JSON_String(json_frame, "\"bun\"", temp_str,
                          sizeof(temp_str))) // broker username
      {
        if (strlen(temp_str) > 40) {
          error_code = 403; /* Invalid range */
        } else if ((strcmp(first.MQTTUsername, temp_str)) != 0) {
          strncpy(first.MQTTUsername, temp_str, sizeof(first.MQTTUsername) - 1);
          first.MQTTUsername[sizeof(first.MQTTUsername) - 1] = '\0';
          f_data_update = 1;
          GSM.F_Server_Change = 1; /* Mark broker credentials changed to trigger reconnect */
        }
      }

      /* "bpw": MQTT Broker Password (Max 40 characters) */
      if (Get_JSON_String(json_frame, "\"bpw\"", temp_str,
                          sizeof(temp_str))) // broker password
      {
        if (strlen(temp_str) > 40) {
          error_code = 403; /* Invalid range */
        } else if ((strcmp(first.MQTTPassword, temp_str)) != 0) {
          strncpy(first.MQTTPassword, temp_str, sizeof(first.MQTTPassword) - 1);
          first.MQTTPassword[sizeof(first.MQTTPassword) - 1] = '\0';
          f_data_update = 1;
          GSM.F_Server_Change = 1; /* Mark broker credentials changed to trigger reconnect */
        }
      }

      /* "bpt": MQTT Broker Port (Valid range: 1 to 65535, typical 1883 or 8883) */
      if (Get_JSON_Int(json_frame, "\"bpt\"", &temp_int)) // broker port
      {
        if (temp_int < 1 || temp_int > 65535) {
          error_code = 403; /* Invalid range */
        } else if (first.MQTTPort != (uint16_t)temp_int) {
          first.MQTTPort = (uint16_t)temp_int;
          f_data_update = 1;
          GSM.F_Server_Change = 1; /* Mark broker credentials changed to trigger reconnect */
        }
      }

      /* "spt": Sensor Sampling & Telemetry Interval in seconds (1 to 9999s) */
      if (Get_JSON_Int(json_frame, "\"spt\"", &temp_int)) {
        if (temp_int < 1 || temp_int > 9999) {
          error_code = 403; /* Invalid range */
        } else if (first.Sample_Time != (uint16_t)temp_int) {
          first.Sample_Time = (uint16_t)temp_int;
          f_data_update = 1;
        }
      }

      //--------------------------------------------------------------------
      /* "bdr": RS485 Baud Rate Index (0:9600, 1:19200, 2:38400, 3:57600, 4:115200) */
      if (Get_JSON_Int(json_frame, "\"bdr\"", &temp_int)) // baudrate
      {
        if (temp_int < 0 || temp_int > 4) {
          error_code = 403; /* Invalid range */
        } else if (first.Baud_Rate != (uint8_t)temp_int) {
          first.Baud_Rate = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "prt": RS485 Parity Bit (1: None, 2: Even, 3: Odd) */
      if (Get_JSON_Int(json_frame, "\"prt\"", &temp_int)) // parity
      {
        if (temp_int < 1 || temp_int > 3) {
          error_code = 403; /* Invalid range */
        } else if (first.Parity != (uint8_t)temp_int) {
          first.Parity = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "spb": RS485 Stop Bits (1: 1 Stop Bit, 2: 2 Stop Bits) */
      if (Get_JSON_Int(json_frame, "\"spb\"", &temp_int)) // Stop_bit
      {
        if (temp_int < 1 || temp_int > 2) {
          error_code = 403; /* Invalid range */
        } else if (first.Stop_bit != (uint8_t)temp_int) {
          first.Stop_bit = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "tts": Total number of Modbus slave devices to poll (1 to 16) */
      if (Get_JSON_Int(json_frame, "\"tts\"", &temp_int)) // total sensors
      {
        if (temp_int < 1 || temp_int > 16) {
          error_code = 403; /* Invalid range */
        } else if (first.No_Of_Slave != (uint8_t)temp_int) {
          first.No_Of_Slave = (uint8_t)temp_int;
          /* Reset cached register buffers and display values to zero upon slave count resize */
          memset(modbus.HoldReg, 0, sizeof(modbus.HoldReg));
          memset(Run.Modbus_Val, 0, sizeof(Run.Modbus_Val));
          for (int i = 0; i < 16; i++) {
              Run.Modbus_Val[i][0] = '0';
              Run.Modbus_Val[i][1] = '\0';
          }
          f_data_update = 1;
        }
      }

      //--------------------------------------------------------------------
      /* "dno": Target Slave Device Index (0 to 15) for setting Modbus poll specs */
      if (Get_JSON_Int(json_frame, "\"dno\"", &temp_int)) // device number
      {
        if (temp_int < 0 || temp_int > 15) {
          error_code = 403; /* Invalid range */
        } else {
          modbus.RdDeviceNo = (uint8_t)temp_int;
        }
      }

      /* "sid": Modbus RTU Slave ID Address (1 to 255) for device `RdDeviceNo` */
      if (Get_JSON_Int(json_frame, "\"sid\"", &temp_int)) // slave id
      {
        if (temp_int < 1 || temp_int > 255) {
          error_code = 403; /* Invalid range */
        } else if (first.SlaveID[modbus.RdDeviceNo] != (uint8_t)temp_int) {
          first.SlaveID[modbus.RdDeviceNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "fc": Modbus Function Code (3: Read Holding Regs, 4: Read Input Regs) */
      if (Get_JSON_Int(json_frame, "\"fc\"", &temp_int)) // function code
      {
        if (temp_int < 3 || temp_int > 4) {
          error_code = 403; /* Invalid range */
        } else if (first.FunctionID[modbus.RdDeviceNo] != (uint8_t)temp_int) {
          first.FunctionID[modbus.RdDeviceNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "ad": Modbus Starting Register Address (0 to 65535) */
      if (Get_JSON_Int(json_frame, "\"ad\"", &temp_int)) // address
      {
        if (temp_int < 0 || temp_int > 65535) {
          error_code = 403; /* Invalid range */
        } else if (first.ModbusAdd[modbus.RdDeviceNo] != (uint16_t)temp_int) {
          first.ModbusAdd[modbus.RdDeviceNo] = (uint16_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "qnt": Quantity of 16-bit Modbus registers to read (1 to 30) */
      if (Get_JSON_Int(json_frame, "\"qnt\"", &temp_int)) // quantity
      {
        if (temp_int < 1 || temp_int > 30) {
          error_code = 403; /* Invalid range */
        } else if (first.BytetoRead[modbus.RdDeviceNo] != (uint8_t)temp_int) {
          first.BytetoRead[modbus.RdDeviceNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      //--------------------------------------------------------------------
      /* "rno": Row / display index (0 to 15) to map the variable for telemetry/display */
      if (Get_JSON_Int(json_frame, "\"rno\"", &temp_int)) // row number
      {
        if (temp_int < 0 || temp_int > 15) {
          error_code = 403; /* Invalid range */
        } else {
          modbus.RowNo = (uint8_t)temp_int;
        }
      }

      /* "reg": Register offset within the slave device (multiplied by 2 for byte offset) */
      if (Get_JSON_Int(json_frame, "\"reg\"", &temp_int)) // register number
      {
        if (temp_int < 0 || temp_int > 30) {
          error_code = 403; /* Invalid range */
        } else {
          modbus.RegNo = (uint8_t)(temp_int * 2);
        }
      }

      /* "vid": Slave ID that supplies data for display row `RowNo` */
      if (Get_JSON_Int(json_frame, "\"vid\"", &temp_int)) // slave id for frame value
      {
        if (temp_int < 1 || temp_int > 255) {
          error_code = 403; /* Invalid range */
        } else {
          modbus.RdSlaveID_2 = (uint8_t)temp_int;
          /* Search through configured slaves to find matching Slave ID and bind address */
          for(uint8_t fd=0; fd<first.No_Of_Slave; fd++)
          {
            if(modbus.RdSlaveID_2 == first.SlaveID[fd])
            {
              uint16_t new_val_add = (uint16_t)((fd*100)+modbus.RegNo);
              if (first.DisValAdd[modbus.RowNo] != new_val_add) {
                first.DisValAdd[modbus.RowNo] = new_val_add;
                f_data_update = 1;
              }
              modbus.HoldAdd[modbus.RowNo] = &modbus.HoldReg[fd][modbus.RegNo];
              break;
            }
          }
        }
      }

      /* "dtp": Data Type (0: INT16, 1: UINT16, 2: INT32, 3: UINT32, 4: FLOAT) */
      if (Get_JSON_Int(json_frame, "\"dtp\"", &temp_int)) // datatype
      {
        if (temp_int < 0 || temp_int > 4) {
          error_code = 403; /* Invalid range */
        } else if (first.DataType[modbus.RowNo] != (uint8_t)temp_int) {
          first.DataType[modbus.RowNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "end": Endianness (0: Big Endian, 1: Little Endian, 2: Big Endian Swap, 3: Little Endian Swap) */
      if (Get_JSON_Int(json_frame, "\"end\"", &temp_int)) // endian
      {
        if (temp_int < 0 || temp_int > 3) {
          error_code = 403; /* Invalid range */
        } else if (first.Endian[modbus.RowNo] != (uint8_t)temp_int) {
          first.Endian[modbus.RowNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      /* "dp": Decimal Point Position for fixed-point formatting (0 to 6 places) */
      if (Get_JSON_Int(json_frame, "\"dp\"", &temp_int)) // decimal point
      {
        if (temp_int < 0 || temp_int > 6) {
          error_code = 403; /* Invalid range */
        } else if (first.Decimal[modbus.RowNo] != (uint8_t)temp_int) {
          first.Decimal[modbus.RowNo] = (uint8_t)temp_int;
          f_data_update = 1;
        }
      }

      //--------------------------------------------------------------------
      /* "pwd": Admin Authorization Password */
      if (Get_JSON_Int(json_frame, "\"pwd\"", &temp_int)) // password
      {
        password = (uint16_t)temp_int;
      }

      /* "sn": Gateway 8-character Serial Number ("GTWXXXXXXXX"). Requires password == 9999 */
      if (Get_JSON_String(json_frame, "\"sn\"", temp_str, sizeof(temp_str))) // serial number
      {
        if (strlen(temp_str) != 8) {
          error_code = 403; /* Serial number must be exactly 8 characters */
        } else if ((password == 9999) && ((strcmp(first.Serial_no, temp_str)) != 0)) {
          strncpy(first.Serial_no, temp_str, sizeof(first.Serial_no) - 1);
          first.Serial_no[sizeof(first.Serial_no) - 1] = '\0';
          f_data_update = 1;
        }
      }

      /* Post-parsing validation and persistence routing */
      if (error_code == 403) {
        /* If an invalid parameter was detected, roll back any changes from SPI Flash */
        if (f_data_update) {
          Set_mem_Read();
        }
        if (GSM.F_Server_Change == 1) {
          GSM.F_Server_Change = 0;
        }
        strcpy(error_msg, "invalid range");
        GSM.M_SendConfig = 20; /* Dispatch error response 403 */
      } else if (f_data_update) {
        /* Persist valid modified parameters to W25Q SPI Flash */
        Set_mem_Write();
        if (GSM.F_Server_Change == 1) {
          GSM.F_Server_Change = 2;  /* Initiate MQTT Broker switchover */
          GSM.NW_Status = 2;        /* Set LED blinking to indicate reconnecting state */
          GSM.M_BrokerChange = 0;
        }
        GSM.M_SendConfig = 10; /* Dispatch success response */
      } else {
        /* No parameters were modified (received duplicate configuration) */
        if (GSM.F_Server_Change == 1) {
          GSM.F_Server_Change = 0;
        }
        error_code = 401;
        strcpy(error_msg, "duplicate data");
        GSM.M_SendConfig = 20; /* Dispatch duplicate error 401 */
      }
    } else {
      error_code = 400;
      strcpy(error_msg, "version error");
      GSM.M_SendConfig = 20; /* Dispatch version mismatch error 400 */
    }

  /* -------------------------------------------------------------------------
   * ACTION: "reset"
   * Erases circular flash log pointers and clears message sequence numbers.
   * ------------------------------------------------------------------------- */
  } else if (strstr(json_frame, "\"act\":\"reset\"") != NULL) {
    strcpy(API_action, "reset");
    if (!f_version) {
      Device_Reset();        /* Clear power-fail flash sector indices */
      GSM.M_SendConfig = 10; /* Dispatch success response */
    } else {
      error_code = 400;
      strcpy(error_msg, "version error");
      GSM.M_SendConfig = 20;
    }
  } 
  /* -------------------------------------------------------------------------
   * ACTION: "get-reading"
   * Triggers an immediate sensor sample reading and MQTT publish dispatch.
   * ------------------------------------------------------------------------- */
  else if (strstr(json_frame, "\"act\":\"get-reading\"") != NULL) {
    strcpy(API_action, "get-reading");
    if (!f_version) {
      if (GSM.F_Send_Sample == 0)
        GSM.F_Send_Sample = 2; /* Force immediate sample transmission */
    } else {
      error_code = 400;
      strcpy(error_msg, "version error");
      GSM.M_SendConfig = 20;
    }
  } 
  /* -------------------------------------------------------------------------
   * UNKNOWN ACTION OR SYNTAX ERROR
   * ------------------------------------------------------------------------- */
  else {
    if (!Get_JSON_String(json_frame, "\"act\"", API_action, sizeof(API_action))) {
      strcpy(API_action, "unknown");
    }
    if (f_version) {
      error_code = 400;
      strcpy(error_msg, "version error");
    } else {
      error_code = 405; /* Command not supported / unrecognized */
      strcpy(error_msg, "command not match");
    }
    GSM.M_SendConfig = 20;
  }
}

/**
 * @brief Periodically handles visual status LEDs for Cellular Network, GPS, and Data/MQTT.
 * 
 * Execution Cadence: Called within GSM_TO_MQTT_FUNC(), but throttled to run exactly
 * once every 100 milliseconds using FreeRTOS tick comparison.
 * 
 * Cycle Timing: Uses a 32-step counter (`cnt` 0 to 31) giving a total period of 3.2 seconds.
 * - NW_LED   (Cellular Network): Flashes ON for 400ms (cnt <= 3) every 3.2s when registered (F_NW=1).
 * - GPS_LED  (GNSS Satellite):   Flashes ON for 400ms (cnt <= 3) every 3.2s when fix acquired (GPS_FIX=1).
 * - DATA_LED (MQTT & Telemetry):
 *     - Fast Toggle (100ms) when NW_Status == 2 (Broker connection in progress or server switching).
 *     - Periodic Flash (400ms ON every 3.2s) when NW_Status == 1 (MQTT connected & active).
 *     - OFF when NW_Status == 0 (Disconnected / Idle).
 */
void GSM_LED_Handler(void) {
  static TickType_t last_tick = 0; /* Timestamp of previous execution */
  static uint8_t cnt = 0;          /* 100ms step counter (0 to 31 -> 3.2s window) */

  if ((xTaskGetTickCount() - last_tick) < pdMS_TO_TICKS(100)) {
    return;
  }
  last_tick = xTaskGetTickCount();

  /* 3.2s total cycle (32 x 100ms): cnt 0-3 = 400ms ON pulse, cnt 4-31 = 2800ms OFF period */
  if (++cnt >= 32) {
    cnt = 0;
  }

  /* NW_LED: ON for 400ms every 3.2s if GSM registered on network (F_NW=1); otherwise OFF */
  if (GSM.F_NW && cnt <= 3) {
    NW_LED_ON();
  } else {
    NW_LED_OFF();
  }

  /* GPS_LED: ON for 400ms every 3.2s if GNSS has valid fix (GPS_FIX=1); otherwise OFF */
  if (GSM.GPS_FIX && cnt <= 3) {
    GPS_LED_ON();
  } else {
    GPS_LED_OFF();
  }

  /* DATA_LED: Fast toggle (100ms) during broker connect/transition (Status=2);
   * Heartbeat pulse (400ms) when connected (Status=1);
   * OFF when disconnected (Status=0) */
  if (GSM.NW_Status == 2) {
    DATA_LED_TOGGLE();
  } else if (GSM.NW_Status == 1 && cnt <= 3) {
    DATA_LED_ON();
  } else {
    DATA_LED_OFF();
  }
}

/**
 * @brief Primary Finite State Machine (FSM) managing Quectel Modem, SIM cards, GNSS, and MQTT session.
 * 
 * State Machine Overview:
 * - Mode 0:      Power-on wait / ready state detection (URC "RDY").
 * - Mode 1-2:    Disable command echo ("ATE0").
 * - Mode 3-5:    Detect active SIM card slot ("AT+QDSIM?").
 * - Mode 6-7:    Check SIM card readiness & PIN status ("AT+CPIN?").
 * - Mode 8-9:    Verify LTE/cellular network registration ("AT+CEREG?").
 * - Mode 10-12:  Retrieve network time ("AT+QLTS=2") and synchronize DS3231 RTC.
 * - Mode 13-14:  Enable GNSS engine power ("AT+QGPS=1").
 * - Mode 15-16:  Open TCP connection to MQTT broker ("AT+QMTOPEN=0,...").
 * - Mode 17-18:  Connect MQTT client with credentials ("AT+QMTCONN=0,...").
 * - Mode 19-20:  Subscribe to configuration topic ("AT+QMTSUB=0,1,.../CONFIG/REQ").
 * - Mode 21-22:  Initiate MQTT publish for telemetry/config ("AT+QMTPUB=0,0,0,0,...").
 * - Mode 23-24:  Transmit JSON payload followed by 0x1A (Ctrl+Z) and verify publish ACK.
 * - Mode 25:     Telemetry queue pointer advancement and loop check.
 * - Mode 34-36:  Disconnect MQTT session ("AT+QMTDISC=0") upon error or server change.
 * - Mode 41-43:  Check signal quality ("AT+CSQ") and perform Dual SIM failover if CSQ <= 9.
 * - Mode 44-46:  Query GNSS RMC sentence ("AT+QGPSGNMEA=\"RMC\"") and extract coordinates.
 * - Mode 71-73:  Switch active SIM slot ("AT+QDSIM=0/1").
 */
void GSM_TO_MQTT_FUNC(void) {
  GSM_LED_Handler(); /* Update LED indicators on every FSM iteration */

  switch (GSM.MODE) {

  /* -------------------------------------------------------------------------
   * MODE 0: MODEM BOOT DETECTION
   * Awaits initial "RDY" unsolicited result code (URC) from the modem on startup.
   * If modem fails to respond within 6000 ticks (~12 seconds), force hard reset.
   * ------------------------------------------------------------------------- */
  case 0: // NW Checking
    ++GSM.Time_count;
    if (GSM.Time_count >= 6000) {
      GSM.Time_count = 0;
      IOT_Reset();
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 1 & 2: DISABLE AT COMMAND ECHO
   * Sends "ATE0\r\n" so the modem does not echo typed characters back into UART RX.
   * Expected response: "\r\nOK\r\n".
   * ------------------------------------------------------------------------- */
  case 1: // ECCO OFF CMD
    IOT_UART_WRITE((uint8_t *)"ATE0\r\n");
    GSMResponseCollect("ATE0\r\r\nOK\r\n", 5);
    break;
  case 2:
    Wait_For_GSMResponse();
    if (strncmp((char *)GSM.Rd_Buff, "\r\nOK\r\n", 6) == 0) {
      GSM.MODE = 3; /* Advance to SIM slot query */
    }
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler(); /* Retry or reset on timeout */
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 3, 4 & 5: DUAL SIM ACTIVE SLOT DETECTION
   * Sends "AT+QDSIM?\r\n" to determine whether SIM 1 (slot 0) or SIM 2 (slot 1) is active.
   * ------------------------------------------------------------------------- */
  case 3: // SIM MODE CHECK CMD
    IOT_UART_WRITE((uint8_t *)"AT+QDSIM?\r\n");
    GSMResponseCollect("\r\n+QDSIM: 1\r\n\r\nOK\r\n", 50);
    GSM.SimNo = 1; // Default assumption: Sim slot-2
    break;
  case 4:
    Wait_For_GSMResponse();
    if (strncmp((char *)GSM.Rd_Buff, "\r\n+QDSIM: 0\r\n\r\nOK\r\n", 19) == 0) {
      GSM.SimNo = 0; // Confirmed: Sim slot-1
      GSM.MODE = 5;
    }
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler();
    }
    break;
  case 5:
    if (GSM.SimNo == 0) // Sim slot-1 is active
    {
      GSM.MODE = 6; // Proceed to check SIM pin/readiness
    } else {
      if (SIM_MODE == 1) // If gateway configured in Single SIM MODE
      {
        GSM.MODE = 71; // Force switch to SIM slot 1
      } else {
        GSM.MODE = 6; // Check readiness of SIM slot 2
      }
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 6 & 7: SIM PIN & CARD READINESS CHECK
   * Sends "AT+CPIN?\r\n". Expected response: "+CPIN: READY".
   * If SIM card is absent or PIN-locked for 5 consecutive retries, triggers failover.
   * ------------------------------------------------------------------------- */
  case 6: // Sim Check CMD
    IOT_UART_WRITE((uint8_t *)"AT+CPIN?\r\n");
    break;
  case 7:
    if (strncmp((char *)GSM.Rd_Buff, "\r\n+CPIN: READY\r\n\r\nOK\r\n", 22) ==
        0) {
      GSM.MODE = 8; /* SIM is ready; proceed to check cellular registration */
    } else {
      GSM.Time_count++;
      if (GSM.Time_count >= 100) {
        GSM.Time_count = 0;
        GSM.Error_cnt++;
        if (GSM.Error_cnt >= 5) {
          if (SIM_MODE == 1) // Single SIM MODE: cannot failover, reset modem
          {
            IOT_Reset();
          } else {
            GSM.MODE = 71; // Dual SIM MODE: switch to alternative SIM slot
          }
        } else {
          GSM.MODE--; /* Retry AT+CPIN? query */
        }
      }
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 8 & 9: CELLULAR NETWORK REGISTRATION
   * Sends "AT+CEREG?\r\n" to query LTE/EPS network registration.
   * Responses:
   * - "+CEREG: 0,1": Registered, home network.
   * - "+CEREG: 0,5": Registered, roaming.
   * Allows up to 60 retries (~6 seconds) before performing hardware modem reset.
   * ------------------------------------------------------------------------- */
  case 8: // NW CHECK CMD
    IOT_UART_WRITE((uint8_t *)"AT+CEREG?\r\n");
    break;
  case 9:
    if (strncmp((char *)GSM.Rd_Buff, "\r\n+CEREG: 0,5\r\n\r\nOK\r\n", 21) ==
        0) {
      GSM.MODE = 10; /* Registered (roaming) */
    } else if (strncmp((char *)GSM.Rd_Buff, "\r\n+CEREG: 0,1\r\n\r\nOK\r\n",
                       21) == 0) {
      GSM.MODE = 10; /* Registered (home network) */
    } else {
      GSM.Time_count++;
      if (GSM.Time_count >= 100) {
        GSM.Time_count = 0;
        GSM.Error_cnt++;
        if (GSM.Error_cnt >= 60) {
          IOT_Reset(); /* Exceeded registration timeout: reset modem */
        } else {
          GSM.MODE--; /* Retry AT+CEREG? query */
        }
      }
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 10, 11 & 12: CELLULAR NETWORK TIME SYNCHRONIZATION
   * Sends "AT+QLTS=2\r\n" to read base-station local time ("YYYY/MM/DD,HH:MM:SS±ZZ").
   * Synchronizes the hardware DS3231 RTC clock with the retrieved network time.
   * ------------------------------------------------------------------------- */
  case 10: // DATE & TIME GET CMD
    GSM.F_NW = 1; /* Cellular network confirmed registered */
    IOT_UART_WRITE((uint8_t *)"AT+QLTS=2\r\n");
    GSMResponseCollect("\r\n+QLTS: ", 5);
    break;
  case 11:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSM.MODE = 12;
    }
    break;
  case 12:
    vTaskDelay(
        pdMS_TO_TICKS(50)); // Wait for rest of QLTS string to arrive over UART
    GSM.Rd_Buff[29] = '\0';
    RTC_WrtFromStr(&GSM.Rd_Buff[12]); /* Update DS3231 RTC from string */
    GSM.MODE = 13;
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 13 & 14: GNSS / GPS ENGINE POWER ACTIVATION
   * Sends "AT+QGPS=1\r\n" to turn on the integrated GNSS receiver.
   * ------------------------------------------------------------------------- */
  case 13: // GPS POWER ON CMD
    IOT_UART_WRITE((uint8_t *)"AT+QGPS=1\r\n");
    GSMResponseCollect("\r\nOK\r\n", 30);
    break;
  case 14:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler();
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 15 & 16: OPEN TCP SOCKET TO MQTT BROKER
   * Sends "AT+QMTOPEN=0,\"<MQTTServer>\",<MQTTPort>\r\n".
   * Expected URC: "+QMTOPEN: 0,0" (Client ID 0 connected successfully).
   * ------------------------------------------------------------------------- */
  case 15: // MQTT BROKER CMD
    sprintf((char *)GSM.Wr_Buff, "AT+QMTOPEN=0,\"%s\",%u\r\n", first.MQTTServer,
            first.MQTTPort);
    IOT_UART_WRITE(GSM.Wr_Buff);
    GSMResponseCollect("\r\nOK\r\n\r\n+QMTOPEN: 0,0\r\n", 3000);
    break;
  case 16:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler();
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 17 & 18: MQTT CLIENT CONNECTION WITH CREDENTIALS
   * Sends "AT+QMTCONN=0,\"GTW<Serial_no>\",\"<MQTTUsername>\",\"<MQTTPassword>\"\r\n".
   * Expected URC: "+QMTCONN: 0,0,0" (Connected, accepted).
   * ------------------------------------------------------------------------- */
  case 17: // MQTT BROKER USERNAME & PASS CMD
    GSM.NW_Status = 2; /* Set LED blinking to indicate connection in progress */
    sprintf((char *)GSM.Wr_Buff, "AT+QMTCONN=0,\"GTW%8s\",\"%s\",\"%s\"\r\n",
            first.Serial_no, first.MQTTUsername, first.MQTTPassword);
    IOT_UART_WRITE(GSM.Wr_Buff);
    GSMResponseCollect("\r\nOK\r\n\r\n+QMTCONN: 0,0,0\r\n", 3000);
    break;
  case 18:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler();
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 19 & 20: SUBSCRIBE TO REMOTE CONFIGURATION TOPIC
   * Subscribes to: "SWA/GTW<Serial_no>/CONFIG/REQ" with QoS 0.
   * Expected URC: "+QMTSUB: 0,1,0,0" (Client 0, Msg ID 1, Success, QoS 0).
   * ------------------------------------------------------------------------- */
  case 19: // MQTT TOPIC SUBSCRIBE CMD
    GSM.NW_Status = 1; /* Connected and operational */
    sprintf((char *)GSM.Wr_Buff,
            "AT+QMTSUB=0,1,\"SWA/GTW%8s/CONFIG/REQ\",0\r\n", first.Serial_no);
    IOT_UART_WRITE(GSM.Wr_Buff);
    GSMResponseCollect("\r\nOK\r\n\r\n+QMTSUB: 0,1,0,0\r\n", 200);
    break;
  case 20:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler(); // If Error Then Go To MQTT Disconnect
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 21 & 22: INITIATE MQTT PUBLISH COMMAND
   * Targets either:
   * - "SWA/GTW<Serial_no>/CONFIG/RES" if responding to a config command.
   * - "SWA/GTW<Serial_no>/READING" for periodic or live sensor telemetry.
   * Modem responds with prompt character '>' indicating it is ready for payload data.
   * ------------------------------------------------------------------------- */
  case 21: // MQTT TOPIC PUBLISH CMD
    if (GSM.F_Server_Change == 3) {
      GSM.F_Server_Change = 0;
      // LCD_Icon_Switch(0x7110, 2);
      GSM.MODE = 70;
    } else {
      if (GSM.M_SendConfig == 2 || GSM.M_SendConfig == 11 ||
          GSM.M_SendConfig == 21) // Config Topic Payload
      {
        sprintf((char *)GSM.Wr_Buff,
                "AT+QMTPUB=0,0,0,0,\"SWA/GTW%8s/CONFIG/RES\"\r\n",
                first.Serial_no);
      } else // Reading Topic Payload
      {
        sprintf((char *)GSM.Wr_Buff,
                "AT+QMTPUB=0,0,0,0,\"SWA/GTW%8s/READING\"\r\n",
                first.Serial_no);
      }
      IOT_UART_WRITE(GSM.Wr_Buff);
      GSMResponseCollect("\r\n>", 10);
    }
    break;
  case 22:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3)
    {
      GSM.MODE = 34; // Disconnect on timeout or prompt failure
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 23 & 24: SEND MQTT PAYLOAD & VERIFY ACK
   * Dispatches the prepared JSON message payload followed by ASCII 0x1A (Ctrl+Z).
   * If historic unsent frames exist in SPI Flash (temp.SendCnt < PF.Massege_No - 1),
   * retrieves the archived frame; otherwise transmits live data from `Send`.
   * Expected URC: "+QMTPUB: 0,0,0" (Published successfully).
   * ------------------------------------------------------------------------- */
  case 23: // DATA TRANSFER CMD TO MQTT
    if (GSM.M_SendConfig == 2) {
      CONFIG_FRAME(GSM.Config_str);
    } else if (GSM.M_SendConfig == 11) {
      SUCCESS_FRAME(API_action);
    } else if (GSM.M_SendConfig == 21) {
      ERROR_FRAME(API_action, error_code, error_msg);
    } else {
      /* Check if backlog frames exist in SPI Flash */
      if (temp.SendCnt < (PF.Massege_No - 1)) {
        uint32_t HeadAdd = temp.SendCnt * DataFrameByte;
        if (flash_mutex != NULL) {
          xSemaphoreTake(flash_mutex, portMAX_DELAY);
        }
        SPI_Flash_Read((uint8_t *)&iot_send, HeadAdd, sizeof(iot_send));
        if (flash_mutex != NULL) {
          xSemaphoreGive(flash_mutex);
        }
      } else {
        iot_send = Send; /* Transmit most recent live telemetry */
      }
      READING_FRAME();
    }
    IOT_UART_WRITE(GSM.MQTTFrame); /* Payload buffer already contains trailing 0x1A Ctrl+Z */
    GSMResponseCollect("\r\nOK\r\n\r\n+QMTPUB: 0,0,0\r\n", 200);
    break;
  case 24:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3)
    {
      GSM.MODE = 34; // Publish failed, abort to MQTT disconnect
    }
    break;

  /* -------------------------------------------------------------------------
   * MODE 25: TRANSMISSION QUEUE POINTER ADVANCE
   * Increments the sent frame counter (`temp.SendCnt`).
   * If additional backlog frames remain in flash, loops back to Mode 21 immediately.
   * If all frames have been dispatched, advances to Mode 41 to inspect CSQ and GNSS.
   * ------------------------------------------------------------------------- */
  case 25:
    if (GSM.M_SendConfig == 0) {
      if (temp.F_Power_On) {
        temp.SendCnt++;
      }
    }
    if (GSM.M_SendConfig == 2 || GSM.M_SendConfig == 11 ||
        GSM.M_SendConfig == 21) {
      GSM.M_SendConfig = 0;
    }
    if (temp.SendCnt < (PF.Massege_No - 1)) {
      GSM.F_ResponseCheck = 0;
      GSM.SimSwitch = 0;
      if (GSM.M_SendConfig == 1) {
        GSM.M_SendConfig = 2;
      } else if (GSM.M_SendConfig == 10) {
        GSM.M_SendConfig = 11;
      } else if (GSM.M_SendConfig == 20) {
        GSM.M_SendConfig = 21;
      }
      GSM.MODE = 21; /* Loop back to publish next frame in backlog */
    } else {
      GSM.MODE = 41; // All pending frames published; proceed to signal check
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 34, 35 & 36: MQTT DISCONNECT & RECOVERY
   * Sends "AT+QMTDISC=0\r\n" to cleanly terminate broker session.
   * Waits for a short cooldown interval (200 ticks), then either reconnects (Mode 15)
   * or triggers a SIM slot swap (Mode 71) if failover was flagged.
   * ------------------------------------------------------------------------- */
  case 34:
    GSM.NW_Status = 0;
    IOT_UART_WRITE((uint8_t *)"AT+QMTDISC=0\r\n");
    GSMResponseCollect("\r\nOK\r\n\r\n+QMTDISC: 0,0\r\n\r\n+QMTSTAT: 0,5\r\n",
                       200);
    break;
  case 35:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSMResponse_Error_Handler();
    }
    break;
  case 36:
    ++GSM.Time_count;
    if (GSM.Time_count >= 200) {
      GSM.Time_count = 0;
      if (GSM.SimSwitch) {
        GSM.MODE = 71; /* Perform Dual SIM swap */
      } else {
        GSM.MODE = 15; /* Re-open MQTT broker session */
      }
    }
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 41, 42 & 43: CELLULAR SIGNAL QUALITY QUERY & DUAL SIM FAILOVER
   * Sends "AT+CSQ\r\n" to read Received Signal Strength Indication (RSSI).
   * - CSQ 0-31: Valid signal levels. Scaled to percentage: (Signal_Val * 100) / 31.
   * - CSQ <= 9 or CSQ == 99 (unknown/poor): Triggers SIM failover if SIM_MODE == 2.
   * ------------------------------------------------------------------------- */
  case 41:
    IOT_UART_WRITE((uint8_t *)"AT+CSQ\r\n");
    GSMResponseCollect("\r\n+CSQ: ", 50);
    break;
  case 42:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      Run.Signal_Strength = 0;
      GSM.MODE = 43;
    }
    break;
  case 43:
    if (GSM.F_ResponseCheck == 2) {
      if (GSM.Rd_Buff[9] == ',') {
        GSM.Signal_Val = (uint8_t)(GSM.Rd_Buff[8] - '0');
      } else {
        GSM.Signal_Val =
            (uint8_t)(((GSM.Rd_Buff[8] - '0') * 10) + (GSM.Rd_Buff[9] - '0'));
      }
      /* If signal is unacceptably low (<= 9) or invalid (99), evaluate Dual SIM failover */
      if (GSM.Signal_Val <= 9 || GSM.Signal_Val == 99) {
        if (SIM_MODE == 2) {
          GSM.SimSwitch = 1;
          GSM.MODE = 71; /* Switch SIM slot */
        }
        if (GSM.Signal_Val == 99) {
          GSM.Signal_Val = 0;
        }
      }
      /* Convert 0-31 range to 0-100% */
      Run.Signal_Strength = (uint8_t)((GSM.Signal_Val * 100) / 31);
    }
    if (!GSM.SimSwitch) {
      GSM.MODE = 44; /* Proceed to GNSS position retrieval */
    }
    break;

  /* -------------------------------------------------------------------------
   * MODE 44, 45 & 46: GNSS NMEA RMC SENTENCE RETRIEVAL
   * Sends "AT+QGPSGNMEA=\"RMC\"\r\n" to query Recommended Minimum Specific GPS Data.
   * Calls ProcessGPSFrame() to parse latitude, longitude, and lock status.
   * ------------------------------------------------------------------------- */
  case 44:
    IOT_UART_WRITE((uint8_t *)"AT+QGPSGNMEA=\"RMC\"\r\n");
    GSMResponseCollect("\r\n+QGPSGNMEA: $GNRMC", 5);
    break;
  case 45:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      GSM.MODE = 70; /* Return to idle if GNSS query times out */
    }
    break;
  case 46:
    ProcessGPSFrame(); /* Parse latitude, longitude, and fix status */
    GSM.MODE = 70;     /* Idle wait state until next sampling trigger */
    break;

  //----------------------------------------------
  /* -------------------------------------------------------------------------
   * MODE 71, 72 & 73: DUAL SIM HARDWARE SLOT SWITCHING
   * Issues "AT+QDSIM=0\r\n" (select SIM 1) or "AT+QDSIM=1\r\n" (select SIM 2).
   * After slot switch completes, jumps back to Mode 6 to verify SIM PIN and register.
   * ------------------------------------------------------------------------- */
  case 71: // SIM SWITCH CMD
    if (GSM.SimNo) {
      IOT_UART_WRITE((uint8_t *)"AT+QDSIM=0\r\n");
    } else {
      IOT_UART_WRITE((uint8_t *)"AT+QDSIM=1\r\n");
    }
    GSMResponseCollect("\r\nOK\r\n", 500);
    break;
  case 72:
    Wait_For_GSMResponse();
    if (GSM.F_ResponseCheck == 3) {
      IOT_Reset(); /* Modem failed to respond to slot switch: perform hard reset */
    }
    break;
  case 73:
    if (GSM.SimNo) {
      GSM.SimNo = 0;
    } else {
      GSM.SimNo = 1;
    }
    GSM.SimSwitch = 0;
    GSM.MODE = 6; /* Restart sequence by checking PIN on the new SIM slot */
    break;

  default:
    break;
  }
}

/**
 * @brief Parses raw NMEA $GNRMC sentence returned by Quectel GNSS engine.
 * 
 * Token Structure in $GNRMC:
 * - Comma 3: Receiver Status ('A' = Data valid / autonomous fix, 'V' = Navigation receiver warning).
 * - Comma 4: Latitude in ddmm.mmmm format.
 * - Comma 5: Latitude Direction ('N' = North, 'S' = South -> negate latitude).
 * - Comma 6: Longitude in dddmm.mmmm format.
 * - Comma 7: Longitude Direction ('E' = East, 'W' = West -> negate longitude).
 */
void ProcessGPSFrame() {
  char Hold_Buff[512];    /* Token buffer for extracting individual comma-delimited fields */
  uint8_t Byte_Cnt = 0;   /* Token character length counter */
  uint8_t Comma = 0;      /* Field index counter tracking NMEA comma separators */

  for (uint8_t rmc = 14; rmc < GSM.Rd_Cnt; rmc++) {
    Hold_Buff[Byte_Cnt] = (char)GSM.Rd_Buff[rmc];
    Byte_Cnt++;
    if (GSM.Rd_Buff[rmc] == ',') {
      Hold_Buff[Byte_Cnt - 1] = '\0'; /* Terminate extracted token string */
      Comma++;
      switch (Comma) {

			case 3 : /* Field 2: Status ('A' = Active / Fix valid, 'V' = Void / No fix) */
				if(Hold_Buff[0]=='A'){GSM.GPS_FIX = 1;}
				else						     {GSM.GPS_FIX = 0;}
				break;

      case 4: /* Field 3: Latitude coordinate string (ddmm.mmmm) */
        if (Hold_Buff[0] != '\0') {
          GPS_DATAConversion(Hold_Buff, (Byte_Cnt - 1), 1);
        } else {
          Run.latitude = 0;
        }
        break;

      case 5: /* Field 4: Latitude hemisphere ('N' or 'S') */
        if (Hold_Buff[0] == 'S') {
          Run.latitude *= -1; /* South coordinates are signed negative */
        }
        break;

      case 6: /* Field 5: Longitude coordinate string (dddmm.mmmm) */
        if (Hold_Buff[0] != '\0') {
          GPS_DATAConversion(Hold_Buff, (Byte_Cnt - 1), 2);
        } else {
          Run.longitude = 0;
        }
        break;

      case 7: /* Field 6: Longitude hemisphere ('E' or 'W') */
        if (Hold_Buff[0] == 'W') {
          Run.longitude *= -1; /* West coordinates are signed negative */
        }
        break;

      default:
        break;
      }
      Byte_Cnt = 0; /* Reset token buffer index for next field */
    }
    if (Comma >= 7) {
      break; /* Stop processing once coordinate fields 1-7 are extracted */
    }
  }
}

/**
 * @brief Converts NMEA coordinate format (ddmm.mmmm or dddmm.mmmm) into decimal degrees.
 * 
 * Algorithm:
 * 1. Locates the decimal point '.' separator in `GpsStr`.
 * 2. Parses pre-decimal digits into integer `pre` and post-decimal digits into integer `frac`.
 * 3. Extracts Degrees: `dd = pre / 100` (last two digits of `pre` belong to minutes).
 * 4. Extracts Minutes: `mm = pre % 100`.
 * 5. Calculates Decimal Degrees: `dd + (mm + (frac / 10^frac_len)) / 60.0f`.
 * 
 * @param GpsStr String containing NMEA coordinate characters.
 * @param CalLen Length of the coordinate string in bytes.
 * @param CalOpr Target destination: 1 = Run.latitude, 2 = Run.longitude.
 */
void GPS_DATAConversion(char *GpsStr, uint8_t CalLen, uint8_t CalOpr) {
  uint8_t dot = CalLen; /* Index of decimal point (defaults to end of string if absent) */
  uint32_t pre = 0;     /* Accumulated digits before '.' (degrees * 100 + whole minutes) */
  uint32_t frac = 0;    /* Accumulated fractional digits after '.' */
  uint32_t dd = 0, mm = 0;
  uint32_t scale = 1;   /* Fractional scale factor (10^number_of_fractional_digits) */

  /* Parse whole number part before the decimal point */
  for (uint8_t i = 0; i < CalLen; i++) {
    char c = GpsStr[i];
    if (c == '.') {
      dot = i;
      break;
    }
    pre = pre * 10u + (uint32_t)(c - '0');
  }

  /* Parse fractional part after the decimal point */
  if (dot < CalLen) {
    for (uint8_t i = (dot + 1); i < CalLen; i++) {
      char c = GpsStr[i];
      if (c < '0' || c > '9')
        break;
      frac = frac * 10u + (uint32_t)(c - '0');
      scale *= 10u;
    }
  }

  switch (CalOpr) {
  case 1: /* Calculate Latitude: dd = pre/100, mm = pre%100 */
    dd = pre / 100;
    mm = pre % 100;
    Run.latitude = ((float)frac / (float)scale);
    Run.latitude += (float)mm;
    Run.latitude /= 60.0f;
    Run.latitude += (float)dd;
    break;

  case 2: /* Calculate Longitude: dd = pre/100, mm = pre%100 */
    dd = pre / 100;
    mm = pre % 100;
    Run.longitude = ((float)frac / (float)scale);
    Run.longitude += (float)mm;
    Run.longitude /= 60.0f;
    Run.longitude += (float)dd;
    break;
  }
}

/**
 * @brief Performs hardware power-on timing sequence for the Quectel modem.
 * 
 * Hardware Sequence:
 * 1. Asserts PB4 HIGH to enable modem VBAT power rail.
 * 2. Waits 100ms for power supply stabilization.
 * 3. Asserts PB3 (PWRKEY) HIGH for 2005ms (exceeding Quectel's minimum >1000ms PWRKEY pulse).
 * 4. De-asserts PB3 LOW to finish power-on pulse.
 */
void IOT_ON(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_4);   /* Enable VBAT supply rail to modem */
  vTaskDelay(pdMS_TO_TICKS(100));

  GPIO_SetBits(GPIOB, GPIO_Pin_3);   /* Assert PWRKEY pin */
  vTaskDelay(pdMS_TO_TICKS(2005));  /* Hold PWRKEY active for >2 seconds */
  GPIO_ResetBits(GPIOB, GPIO_Pin_3); /* Release PWRKEY pin */
}

/**
 * @brief Resets cellular state machine and sends software reboot AT command.
 * 
 * Clears network registration, packet counters, and signal strengths,
 * then sends "AT+CFUN=1,1\r\n" requesting a full modem baseband software restart.
 */
void IOT_Reset(void) {  
  GSM.F_NW = 0;             /* Clear network registered flag */
  GSM.Rd_Cnt = 0;           /* Reset RX buffer counter */
  GSM.MODE = 0;             /* Return to initial state: await modem boot */
  GSM.Time_count = 0;       /* Reset tick counter */
  GSM.Error_cnt = 0;        /* Reset error retry counter */
  GSM.F_Error = 0;
  GSM.NW_Status = 0;        /* Turn off connection LED */
  GSM.Signal_Val = 0;
  Run.Signal_Strength = 0;
  IOT_UART_WRITE((uint8_t *)"AT+CFUN=1,1\r\n"); /* Request modem software reboot */
}

/**
 * @brief Manages retry and error escalation for failed AT command responses.
 * 
 * Increments `Error_cnt`. If fewer than 2 retries failed, rolls back to previous
 * mode (`GSM.MODE--`) to repeat the AT command. If 2 retries failed, triggers `IOT_Reset()`.
 */
void GSMResponse_Error_Handler(void) {
  GSM.Error_cnt++;
  if (GSM.Error_cnt >= 2) {
    IOT_Reset(); /* Two consecutive command failures: reset modem */
  } else {
    GSM.MODE--;  /* Retry previous command */
  }
}

/**
 * @brief Non-blocking checker comparing incoming UART data against expected response string.
 * 
 * Verifies if `GSM.Rd_Buff` matches `GSM.RdCheckBuff` via `strncmp` or `strstr`.
 * - On match: Sets `F_ResponseCheck = 2` and increments `GSM.MODE++`.
 * - On timeout: Sets `F_ResponseCheck = 3` after `ResponseCheckTM` ticks elapse.
 */
void Wait_For_GSMResponse(void) {
  if (strncmp((char *)GSM.Rd_Buff, (char *)GSM.RdCheckBuff,
              (unsigned int)GSM.RdCheckLen) == 0 ||
      strstr((char *)GSM.Rd_Buff, (char *)GSM.RdCheckBuff) != NULL) {
    GSM.Time_count = 0;
    GSM.F_ResponseCheck = 2; /* Expected response confirmed */
    GSM.MODE++;              /* Advance to next state */
  }

  ++GSM.Time_count;
  if (GSM.Time_count >= GSM.ResponseCheckTM) {
    GSM.Time_count = 0;
    GSM.F_ResponseCheck = 3; /* Response timeout occurred */
  }
}

/**
 * @brief Configures expected response pattern and timeout limit for an AT command.
 * 
 * @param CheckBuff Pointer to expected substring (e.g. "\r\nOK\r\n" or "+QMTOPEN: 0,0").
 * @param CheckTm Timeout period in FSM iterations / ticks.
 */
void GSMResponseCollect(char *CheckBuff, uint16_t CheckTm) {
  GSM.ResponseCheckTM = CheckTm;
  strncpy((char *)GSM.RdCheckBuff, CheckBuff, sizeof(GSM.RdCheckBuff) - 1);
  GSM.RdCheckLen = (uint16_t)strlen((char *)GSM.RdCheckBuff);
  GSM.Time_count = 0;
  GSM.F_ResponseCheck = 1; /* Arm response verification */
}

/**
 * @brief Enqueues a single character into the non-blocking Quectel TX ring buffer.
 * 
 * Flow Control:
 * If the circular ring buffer is full, waits up to 50ms with interrupts enabled,
 * yielding CPU time to prevent blocking higher-priority FreeRTOS tasks.
 * 
 * @param ch ASCII character byte to send.
 */
void Quectel_SendByte(uint8_t ch) {
  uint16_t next_head = (quectel_tx_head + 1) % QUECTEL_TX_BUF_SIZE;
  uint32_t wait_ticks = 50;

  /* If ring buffer is full, wait up to 50ms for ISR to drain space */
  while (next_head == quectel_tx_tail && wait_ticks > 0) {
    taskENTER_CRITICAL();
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE); /* Ensure TXE interrupt is armed */
    taskEXIT_CRITICAL();

    vTaskDelay(1);
    wait_ticks--;
  }

  if (next_head == quectel_tx_tail) {
    return; /* Dropped byte if buffer remains saturated after timeout */
  }

  quectel_tx_buf[quectel_tx_head] = ch;
  quectel_tx_head = next_head;

  /* Arm USART2 TXE interrupt to trigger ring buffer draining */
  taskENTER_CRITICAL();
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  taskEXIT_CRITICAL();
}

/**
 * @brief Transmits a null-terminated AT command or payload string to Quectel Modem.
 * 
 * Clears the RX buffer (`GSM.Rd_Buff`) and pushes every character of `gsm_msg`
 * into the non-blocking TX ring buffer. Then increments `GSM.MODE++` to await response.
 * 
 * @param gsm_msg Pointer to null-terminated command string.
 */
void IOT_UART_WRITE(uint8_t *gsm_msg) {
  uint16_t gsm_msg_len = 0;
  if (!gsm_msg)
    return;
  gsm_msg_len = strlen((char *)gsm_msg);
  if (gsm_msg_len == 0)
    return;

  /* Flush RX buffer prior to issuing new command */
  memset(GSM.Rd_Buff, 0, sizeof(GSM.Rd_Buff));
  GSM.Rd_Cnt = 0;

  /* Feed characters into circular transmit buffer */
  for (uint16_t i = 0; i < gsm_msg_len; i++) {
    Quectel_SendByte(gsm_msg[i]);
  }

  GSM.MODE++; /* Advance state to response-wait mode */
}

/**
 * @brief USART2 Interrupt Service Routine for Cellular Modem communication.
 * 
 * Functions:
 * 1. RXNE (Receiver Not Empty):
 *    - Captures incoming characters into `GSM.Rd_Buff`.
 *    - Detects modem boot banner ("RDY") in Mode 0.
 *    - Extracts streaming JSON frames by tracking nested curly braces ('{' and '}').
 *      When brace count balances to zero, flags `GSM.json_ready_flag = 1`.
 *    - Clears Overrun Error (ORE) flag if received.
 * 2. TXE (Transmit Data Register Empty):
 *    - Drains characters from circular buffer `quectel_tx_buf` into `USART2->DATAR`.
 *    - Automatically disables TXE interrupt once the buffer becomes empty.
 */
void USART2_IRQHandler(void) __attribute__((interrupt()));
void USART2_IRQHandler(void) {
  /* Handle incoming byte reception */
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
    uint8_t uart_char = USART_ReceiveData(USART2);

    /* Store into primary response buffer */
    GSM.Rd_Buff[GSM.Rd_Cnt++] = uart_char;
    if (GSM.Rd_Cnt >= sizeof(GSM.Rd_Buff)) {
      GSM.Rd_Cnt = 0; /* Prevent buffer overflow */
    }

    /* Boot detection: Detect "RDY" message from modem power-up */
    if (GSM.MODE == 0) {
      if (uart_char == '\n') {
        if (GSM.Rd_Buff[0] == 'R' && GSM.Rd_Buff[1] == 'D' &&
            GSM.Rd_Buff[2] == 'Y') {
          GSM.MODE = 1; /* Modem is ready; advance to ATE0 */
        }
        GSM.Rd_Cnt = 0;
      }
    }

    /* JSON Command Parser: count braces to identify complete JSON object */
    if (uart_char == '{') {
      GSM.brace_count++;
      if (!GSM.F_NewFrame) {
        GSM.Rd_Cnt2 = 0;
        GSM.F_NewFrame = 1; /* Mark start of new JSON frame */
      }
    }
    if (GSM.F_NewFrame == 1) {
      GSM.CMD_Buff[GSM.Rd_Cnt2++] = uart_char;
      if (GSM.Rd_Cnt2 >= (sizeof(GSM.CMD_Buff) - 1)) {
        GSM.Rd_Cnt2 = 0; /* Safety overflow guard */
      }
      if (uart_char == '}') {
        GSM.brace_count--;
        if (GSM.brace_count <= 0) {
          GSM.brace_count = 0;
          GSM.CMD_Buff[GSM.Rd_Cnt2] = '\0'; /* Terminate JSON string */
          GSM.json_ready_flag = 1;          /* Signal JSON frame available */
          GSM.F_NewFrame = 0;
        }
      }
    }
  } else if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) {
    USART_ReceiveData(USART2); /* Clear overrun error flag by reading data register */
  }

  /* Handle transmit data register empty: drain TX ring buffer */
  if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
    if (quectel_tx_tail != quectel_tx_head) {
      USART_SendData(USART2, quectel_tx_buf[quectel_tx_tail]);
      quectel_tx_tail = (quectel_tx_tail + 1) % QUECTEL_TX_BUF_SIZE;
    } else {
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE); /* Buffer empty: disable TXE IRQ */
    }
  }
}

/**
 * @brief Initializes USART2 peripheral and GPIO pins for Quectel modem communication.
 * 
 * Hardware Configuration:
 * - PA2: USART2 TX (Alternate Function Push-Pull, 50MHz).
 * - PA3: USART2 RX (Input Floating).
 * - Baud Rate: 115200 bps, 8 Data Bits, 1 Stop Bit, No Parity, No Hardware Flow Control.
 * - NVIC: USART2_IRQn enabled with Preemption Priority 1, Subpriority 1.
 */
void Quectel_Init(void) {

  if (json_mutex == NULL) {
    json_mutex = xSemaphoreCreateMutex();
  }
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  USART_InitTypeDef USART_InitStructure = {0};
  NVIC_InitTypeDef NVIC_InitStructure = {0};

  /* Enable peripheral clocks for GPIOA and USART2 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* PA2 (USART2 TX) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* PA3 (USART2 RX) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 parameters */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(USART2, &USART_InitStructure);

  /* Enable USART2 Receive Interrupt */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Configure NVIC for USART2 */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable USART2 peripheral */
  USART_Cmd(USART2, ENABLE);
}