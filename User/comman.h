#ifndef __COMMAN_H
#define __COMMAN_H

#include "FreeRTOS.h"
#include "ch32v20x.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

/* ==============================================================================
 * LED HARDWARE PIN DEFINITIONS
 * ============================================================================== */
#define DATA_LED GPIO_Pin_0   // PB0: DATA activity LED (toggles on MQTT publish / network activity)
#define GPS_LED GPIO_Pin_15   // PA15: GPS fix status LED (solid/blink when GPS is locked)
#define NW_LED GPIO_Pin_1     // PA1: Cellular Network LED (indicates SIM / 4G registration)
#define PWR_LED GPIO_Pin_1    // PB1: System Power LED (illuminated when MCU is powered)

/* ==============================================================================
 * LED FAST BIT OPERATIONS (Direct Register Access via BSHR, BCR, and OUTDR)
 * - BSHR (Bit Set/Reset Register): Writing 1 to bits 0-15 sets the pin HIGH.
 * - BCR (Bit Clear Register): Writing 1 clears the pin LOW.
 * - OUTDR (Output Data Register): XOR toggles the pin state.
 * ============================================================================== */
#define DATA_LED_ON()     (GPIOB->BSHR = DATA_LED)
#define DATA_LED_OFF()    (GPIOB->BCR  = DATA_LED)
#define DATA_LED_TOGGLE() (GPIOB->OUTDR ^= DATA_LED)

#define GPS_LED_ON()      (GPIOA->BSHR = GPS_LED)
#define GPS_LED_OFF()     (GPIOA->BCR  = GPS_LED)
#define GPS_LED_TOGGLE()  (GPIOA->OUTDR ^= GPS_LED)

#define NW_LED_ON()       (GPIOA->BSHR = NW_LED)
#define NW_LED_OFF()      (GPIOA->BCR  = NW_LED)
#define NW_LED_TOGGLE()   (GPIOA->OUTDR ^= NW_LED)

#define PWR_LED_ON()      (GPIOB->BSHR = PWR_LED)
#define PWR_LED_OFF()     (GPIOB->BCR  = PWR_LED)
#define PWR_LED_TOGGLE()  (GPIOB->OUTDR ^= PWR_LED)

/**
 * @brief SIM Operation Mode:
 * - 1 = Single SIM Mode (default): uses primary SIM slot.
 * - 2 = Dual SIM Automatic Failover Mode: switches between SIM 1 and SIM 2 if signal is poor.
 */
extern uint8_t SIM_MODE;

/**
 * @brief Volatile runtime state variables (not saved to flash).
 */
typedef struct {
  uint32_t SendCnt;            /**< Current sequence/record counter being sent to MQTT */
  volatile uint16_t g_sqw_flag;/**< Set to 1 every second by DS3231 RTC 1Hz EXTI0 interrupt */
  bool F_Power_On;             /**< Flag indicating initial boot/power-on stabilization is complete */
} tempery;

/**
 * @brief Power-Fail (PF) Memory Structure.
 * Saved to SPI Flash (PF_MEM_Sector) to preserve offline data logging pointers across reboots.
 */
typedef struct {
  uint32_t sector_addr; /**< Byte offset address of current active SPI Flash sector */
  uint32_t flash_Add;   /**< Absolute byte address in SPI Flash where next frame will be written */
  uint32_t Rd_Frame_idx;/**< Total number of valid uncorrupted frames recorded in flash */
  uint32_t Massege_No;  /**< Monotonically increasing message/record counter */
  uint16_t frame_idx;   /**< Frame index within current 4096-byte sector (0 to 15) */
  uint16_t sector;      /**< Current sector index in SPI Flash (0 to LastSect) */
  uint8_t FlashFull;    /**< Set to 1 when flash ring buffer wraps around oldest sectors */
  uint8_t FileNo;       /**< Reserved file index number */
  uint8_t Verify_PF;    /**< Magic number (30) to verify flash sector initialization */
} powerfailmem;

/**
 * @brief System Non-Volatile Configuration Structure.
 * Saved to SPI Flash (First_MEM_Sector) to store gateway settings, Modbus maps, and MQTT credentials.
 */
typedef struct {
  uint16_t Sample_Time;       /**< Telemetry sampling interval in seconds (default: 2) */
  uint16_t MQTTPort;          /**< MQTT Broker TCP port (default: 1883) */
  uint16_t ModbusAdd[16];     /**< Modbus starting register address for each of 16 slave slots */
  uint16_t BytetoRead[16];    /**< Number of registers to read for each slave (1 to 30) */
  uint16_t DisValAdd[16];     /**< Internal pointer/offset index for displayed variables */
  uint8_t Verify1;            /**< Magic number (30) used to check if flash contains valid config */
  uint8_t Parity;             /**< RS-485 Parity: 1=None, 2=Even, 3=Odd */
  uint8_t Baud_Rate;          /**< RS-485 Baud rate: 0=4800, 1=9600, 2=19200, 3=38400, 4=115200 */
  uint8_t Stop_bit;           /**< RS-485 Stop bits: 1=1 bit, 2=2 bits */
  uint8_t No_Of_Slave;        /**< Total active Modbus slaves configured (1 to 16) */
  uint8_t SlaveID[16];        /**< Modbus Slave Address (1 to 255) for each slot */
  uint8_t FunctionID[16];     /**< Modbus Function Code (3=Holding Regs, 4=Input Regs) */
  uint8_t DataType[16];       /**< Data Type (0:U16, 1:I16, 2:U32, 3:I32, 4:Float) */
  uint8_t Endian[16];         /**< Byte order / Endianness (0:ABCD, 1:DCBA, 2:BADC, 3:CDAB) */
  uint8_t Decimal[16];        /**< Decimal places for formatting (0 to 6) */
  char Device_ID[17];         /**< User-defined Gateway Name string (max 16 chars + null terminator) */
  char Serial_no[9];          /**< Hardware Serial Number string (8 digits + null terminator, e.g. "26100001") */
  char MQTTServer[41];        /**< MQTT Broker domain/IP address (max 40 chars + null terminator) */
  char MQTTUsername[41];      /**< MQTT Broker authentication username */
  char MQTTPassword[41];      /**< MQTT Broker authentication password */
} memory;

/**
 * @brief Telemetry Frame Data Structure (272 bytes active data, aligned to 256/300 byte flash frames).
 * Holds live sensor values, GNSS location, timestamp, and cellular signal strength.
 */
typedef struct {
  float latitude;          /**< GPS Latitude coordinate in decimal degrees */
  float longitude;         /**< GPS Longitude coordinate in decimal degrees */
  uint8_t day;             /**< Day of week (1 to 7) */
  uint8_t date;            /**< Day of month (1 to 31) */
  uint8_t month;           /**< Month of year (1 to 12) */
  uint8_t year;            /**< 2-digit Year (e.g. 26 for 2026) */
  uint8_t hour;            /**< Hour in 24-hour format (0 to 23) */
  uint8_t minute;          /**< Minute (0 to 59) */
  uint8_t second;          /**< Second (0 to 59) */
  uint8_t Signal_Strength; /**< Cellular RSSI percentage (0 to 100%) */
  char Modbus_Val[16][15]; /**< 16 ASCII string representations of parsed Modbus sensor values */
} Run_Send_t;
// total bytes are : 272 so DataFrameByte is 256/300

/* Global instances of shared structures */
extern tempery temp;
extern memory first;
extern powerfailmem PF;
extern Run_Send_t Run, Send;

/**
 * @brief Resets power-fail flash indices and counters to initial state (record 1).
 */
void Device_Reset(void);

/**
 * @brief Writes the `first` system configuration structure to SPI Flash (Sector First_MEM_Sector).
 */
void Set_mem_Write(void);

/**
 * @brief Reads the `first` system configuration from SPI Flash.
 * If magic number does not match, populates factory default settings and writes to flash.
 */
void Set_mem_Read(void);

/**
 * @brief Writes the `PF` (power-fail memory) structure to SPI Flash (Sector PF_MEM_Sector).
 */
void PF_mem_Write(void);

/**
 * @brief Reads the `PF` (power-fail memory) structure from SPI Flash.
 * If uninitialized, formats indices and writes initial state to flash.
 */
void PF_mem_Read(void);

/**
 * @brief Configures GPIO pins for the status LEDs (PA1, PA15, PB0, PB1) and modem power pins (PB3, PB4).
 */
void gpio_pin_init(void);

/**
 * @brief Lightweight zero-allocation helper to parse string values from a JSON string.
 * @param json Pointer to incoming JSON string.
 * @param key Key to search for (including quotes, e.g. "\"dn\"").
 * @param dest Output destination buffer.
 * @param max_len Maximum length of destination buffer.
 * @return true if key was found and extracted; false otherwise.
 */
bool Get_JSON_String(const char *json, const char *key, char *dest, size_t max_len);

/**
 * @brief Lightweight zero-allocation helper to parse integer values from a JSON string.
 * @param json Pointer to incoming JSON string.
 * @param key Key to search for (e.g. "\"bpt\"").
 * @param dest Pointer to destination int32_t variable.
 * @return true if key was found and converted; false otherwise.
 */
bool Get_JSON_Int(const char *json, const char *key, int32_t *dest);

#endif /* __COMMAN_H */
