#ifndef __BSP_W25Q32_H
#define __BSP_W25Q32_H

#include "ch32v20x.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * @brief Mutex guarding SPI1 and Winbond Flash operations.
 * Prevents race conditions between:
 * 1. `iot_task`: Background telemetry logging and power-fail pointer updates.
 * 2. `usb_cdc_task`: High-speed batch CSV extraction over USB.
 */
extern SemaphoreHandle_t flash_mutex;

/* ==============================================================================
 * SPI FLASH CHIP SELECT (CS) PIN CONFIGURATION (PA4)
 * ============================================================================== */
#define W25Q32_CS_PORT  GPIOA
#define W25Q32_CS_PIN   GPIO_Pin_4

#define W25Q32_CS_LOW()   GPIO_ResetBits(W25Q32_CS_PORT, W25Q32_CS_PIN) /**< Assert SPI Flash CS (Active Low) */
#define W25Q32_CS_HIGH()  GPIO_SetBits(W25Q32_CS_PORT, W25Q32_CS_PIN)   /**< De-assert SPI Flash CS (Idle High) */

/* ==============================================================================
 * FLASH RING BUFFER CAPACITY & FRAME SIZE
 * ============================================================================== */
#define TotalFrameinflash 131000 /**< Maximum historical frames stored in flash (~131,000 frames) */
#define DataFrameByte  256       /**< Byte size allocated per telemetry record (256 bytes) */

/* Winbond Manufacturer and Device ID Constants */
#define W25Q16  0XEF15 /**< Winbond 16M-bit  (2 MByte, 512 sectors of 4KB) */
#define W25Q32  0XEF16 /**< Winbond 32M-bit  (4 MByte, 1024 sectors of 4KB) */
#define W25Q64  0XEF17 /**< Winbond 64M-bit  (8 MByte, 2048 sectors of 4KB) */
#define W25Q128 0XEF18 /**< Winbond 128M-bit (16 MByte, 4096 sectors of 4KB) */
#define W25Q256 0XEF19 /**< Winbond 256M-bit (32 MByte, 8192 sectors of 4KB) */

/**
 * @brief Active Flash Chip mounted on board: W25Q256 (32 MB)
 */
#define WinbondFlash W25Q256

/* ==============================================================================
 * SECTOR MEMORY MAP CONFIGURATION
 * ==============================================================================
 * Each sector is 4096 bytes (4 KB).
 * - Data Sectors (0 to LastSect): Ring buffer for historical telemetry frames.
 * - Configuration Sector (First_MEM_Sector): Stores `memory first` system settings.
 * - Power-Fail Sector (PF_MEM_Sector): Stores `powerfailmem PF` pointers.
 */
#if WinbondFlash == 0XEF15
#define LastSect 509
#define First_MEM_Sector 510
#define PF_MEM_Sector 511
#elif WinbondFlash == 0XEF16
#define LastSect 1021
#define First_MEM_Sector 1022
#define PF_MEM_Sector 1023
#elif WinbondFlash == 0XEF17
#define LastSect 2045
#define First_MEM_Sector 2046
#define PF_MEM_Sector 2047
#elif WinbondFlash == 0XEF18
#define LastSect 4093
#define First_MEM_Sector 4094
#define PF_MEM_Sector 4095
#elif WinbondFlash == 0XEF19
#define LastSect 8189            /**< 0 to 8189 = 8190 sectors dedicated to data frames */
#define First_MEM_Sector 8190    /**< Sector 8190: System Configuration */
#define PF_MEM_Sector 8191       /**< Sector 8191: Power-Fail Index Sector */
#endif

/* Compile-time sanity check to prevent overflow of data sectors */
#if TotalFrameinflash > (LastSect * (4096 / DataFrameByte))
#error >> that much TotalFrameinflash not possible
#endif

/* Prototypes */

/**
 * @brief Commits the staged `Send` telemetry frame to the active SPI Flash address.
 * Advances frame index, updates power-fail pointers, and writes to flash under mutex protection.
 */
void FRAME_WRITE_TO_ExFLASH(void);

/**
 * @brief Initializes SPI1 peripheral, GPIO pins (PA4-CS, PA5-SCK, PA6-MISO, PA7-MOSI), and flash mutex.
 */
void W25QXX_Init(void);

/**
 * @brief Reads the 16-bit Manufacturer and Device ID from the SPI Flash (e.g. 0xEF19 for W25Q256).
 * @return uint16_t JEDEC/Manufacturer ID.
 */
uint16_t W25Q32_ReadID(void);

/**
 * @brief Reads arbitrary number of bytes starting from `ReadAddr` in SPI Flash.
 * @param pBuffer Pointer to destination memory buffer.
 * @param ReadAddr Absolute 24-bit/32-bit byte address in flash.
 * @param NumByteToRead Number of bytes to read.
 */
void SPI_Flash_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

/**
 * @brief Writes up to 256 bytes within a single 256-byte page in SPI Flash.
 * @param pBuffer Pointer to source data buffer.
 * @param WriteAddr Absolute starting byte address (must be page aligned if writing 256 bytes).
 * @param NumByteToWrite Number of bytes to write (1 to 256).
 */
void W25Q32_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/**
 * @brief Writes arbitrary data to flash without checking page boundaries (caller ensures space).
 * @param pBuffer Pointer to data buffer.
 * @param WriteAddr Absolute flash write address.
 * @param NumByteToWrite Number of bytes to write.
 */
void SPI_Flash_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/**
 * @brief Erases a 4096-byte (4KB) sector in SPI Flash.
 * @param Dst_Addr Sector index or byte address within the sector to erase.
 */
void SPI_Flash_Erase_Sector(uint32_t Dst_Addr);

#endif /* __BSP_W25Q32_H */

