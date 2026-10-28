#ifndef __DS3231_H
#define __DS3231_H

#include "ch32v20x.h"
#include "comman.h"

/**
 * @brief DS3231 7-bit I2C Slave Address shifted left by 1 bit for 8-bit R/W format.
 * - Standard 7-bit I2C address of DS3231 is 0x68 (b1101000).
 * - Shifted left: (0x68 << 1) = 0xD0 (Write address: 0xD0, Read address: 0xD1).
 */
#define DS3231_ADDRESS 0xD0 // 0x68 << 1

/**
 * @brief Structure representing calendar date and 24-hour time for the DS3231 RTC chip.
 * Used for storing decoded decimal time values.
 */
typedef struct {
    uint8_t Seconds; /**< Seconds register value (0 to 59) */
    uint8_t Minutes; /**< Minutes register value (0 to 59) */
    uint8_t Hours;   /**< Hours in 24-hour format (0 to 23) */
    uint8_t Day;     /**< Day of week (1 to 7, where 1 = Sunday/Monday depending on config) */
    uint8_t Date;    /**< Day of month (1 to 31) */
    uint8_t Month;   /**< Month of year (1 to 12) */
    uint8_t Year;    /**< Year 2-digit offset (0 to 99, represents 2000-2099) */
} DS3231_TimeTypeDef;

/**
 * @brief Initializes I2C1 peripheral and GPIO pins (PB6=SCL, PB7=SDA) for DS3231 communication.
 */
void DS3231_Init(void);

/**
 * @brief Writes new date/time values from a Run_Send_t structure into DS3231 RTC registers.
 * Converts decimal values into BCD format before sending over I2C.
 * @param time Pointer to Run_Send_t structure holding year, month, date, hour, minute, second.
 */
void ds3231_set_time(Run_Send_t *time);

/**
 * @brief Reads current date and time registers from DS3231 over I2C and stores into Run_Send_t.
 * Converts raw BCD register bytes into decimal format.
 * @param time Pointer to Run_Send_t structure where decoded time is populated.
 */
void ds3231_get_time(Run_Send_t *time);

/**
 * @brief Parses an ASCII time string (e.g. from Quectel AT+QLTS network time or JSON command)
 * and updates the DS3231 RTC.
 * Expected format: "YY/MM/DD,HH:MM:SS" or similar 2-digit pairs.
 * @param StrTime Pointer to ASCII string buffer containing formatted date and time digits.
 */
void RTC_WrtFromStr(uint8_t *StrTime);

/**
 * @brief Configures DS3231 Control Register (0x0E) to output a 1 Hz Square Wave (SQW) on the INT/SQW pin.
 * Clears INTCN bit to enable square wave mode with RS2=0, RS1=0 (1 Hz frequency).
 */
void DS3231_Enable1Hz(void);

/**
 * @brief Configures external interrupt line (EXTI0 on PA0) connected to DS3231 SQW pin.
 * Sets up falling-edge trigger so every 1-second pulse generates an interrupt for system timing.
 */
void DS3231_INT_Init(void);

/**
 * @brief Volatile flag set when RTC needs to be read or periodic timer elapses.
 */
extern volatile uint8_t rtc_read_flag;

#endif /* __DS3231_H */
