#include "w25qxx.h"
#include "comman.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "quectel_ioc.h"

/* Mutex handle for serializing SPI1 bus operations */
SemaphoreHandle_t flash_mutex = NULL;

/* ==============================================================================
 * WINBOND W25Q SERIES SPI FLASH COMMAND OPCODES
 * ============================================================================== */
#define W25X_WriteEnable        0x06 /**< Sets Write Enable Latch (WEL) bit in status register */
#define W25X_WriteDisable       0x04 /**< Resets WEL bit */
#define W25X_ReadStatusReg      0x05 /**< Reads Status Register 1 (bit 0 is BUSY flag) */
#define W25X_FastReadData       0x0B /**< Fast Read command (requires 1 dummy byte at high clock speeds) */
#define W25X_PageProgram        0x02 /**< Programs up to 256 bytes into a single page */
#define W25X_SectorErase        0x20 /**< Erases all 4096 bytes (4KB) in the specified sector to 0xFF */
#define W25X_ManufactDeviceID   0x90 /**< Reads Manufacturer ID (0xEF) and Device ID (e.g. 0x19) */

void FlashSectorMove(uint32_t Sect);
static void W25Q32_Write_SR(uint8_t sr);
static void W25Q32_Wait_Busy(void);
static void W25Q32_Write_Enable(void);
static uint8_t W25Q32_ReadSR(void);

/**
 * @brief Copies an entire 4KB sector from `Sect` to `LastSect`.
 * Used during ring-buffer rollover when `FlashFull == 1` to back up sector records
 * before the active sector is erased and re-used.
 * 
 * @param Sect Source sector index to copy from.
 */
void FlashSectorMove(uint32_t Sect)
{
  uint32_t SecAdd = Sect * 4096;
  uint32_t LastSecAdd;
  uint16_t lp;
  static uint8_t Data_Frame[DataFrameByte];

  /* 1. Erase destination backup sector */
  SPI_Flash_Erase_Sector(LastSect);

  /* 2. Copy frame by frame (16 frames of 256 bytes = 4096 bytes) */
  for (lp = 0; lp < (4096 / DataFrameByte); lp++)
  {
    LastSecAdd = (uint32_t)((LastSect * 4096) + (lp * DataFrameByte));
    SPI_Flash_Read(Data_Frame, SecAdd, DataFrameByte);
    vTaskDelay(2); // Yield brief tick delay
    SPI_Flash_Write_NoCheck(Data_Frame, LastSecAdd, DataFrameByte); 
    SecAdd += DataFrameByte;
  }
}

/**
 * @brief Saves the current staged telemetry snapshot (`Send`) into the external SPI Flash ring buffer.
 * 
 * Execution Steps:
 * 1. Acquires `flash_mutex` to ensure thread-safe SPI1 bus access.
 * 2. Checks if current sector is full (16 frames x 256 bytes = 4096 bytes).
 *    If full, increments sector number, backs up if needed, erases next sector,
 *    and resets `frame_idx = 0`.
 * 3. Calculates absolute destination byte address (`PF.flash_Add`).
 * 4. Programs `sizeof(Send)` bytes of the `Send` telemetry structure into flash.
 * 5. Increments `PF.frame_idx`, total message counter `PF.Massege_No`, and `PF.Rd_Frame_idx`.
 * 6. Checks for overall flash full capacity (`TotalFrameinflash = 131,000`).
 *    If capacity reached, sets `PF.FlashFull = 1` and wraps pointers back to record 1.
 * 7. Releases `flash_mutex`.
 * 8. Commits updated pointers to power-fail flash sector via PF_mem_Write().
 */
void FRAME_WRITE_TO_ExFLASH(void)
{
  if (flash_mutex != NULL && xSemaphoreTake(flash_mutex, portMAX_DELAY) == pdTRUE)
  {
      /* Check if current 4KB sector has reached maximum frame capacity (16 frames of 256 bytes) */
      if (PF.frame_idx >= (4096 / DataFrameByte))
      {
        PF.sector++;
        if (PF.FlashFull == 1)
        {
          FlashSectorMove(PF.sector); // Back up sector before erasing
        }
        SPI_Flash_Erase_Sector(PF.sector);
        PF.sector_addr = (uint32_t)(PF.sector * 4096);
        PF.frame_idx = 0; // Reset frame offset in new sector
      }
      PF.flash_Add = (PF.sector_addr + (PF.frame_idx * DataFrameByte));

      /* Handle initial address zero sector erase */
      if (PF.flash_Add <= 0)
      {
        if (PF.FlashFull == 1)
        {
          FlashSectorMove(0);
        }
        SPI_Flash_Erase_Sector(0);
      }

      /* Write active telemetry frame to flash */
      SPI_Flash_Write_NoCheck((uint8_t *)&Send, PF.flash_Add, sizeof(Send));

      /* Advance pointers and counters */
      PF.frame_idx++;
      PF.Massege_No++;
      if (PF.Rd_Frame_idx < TotalFrameinflash)
      {
        PF.Rd_Frame_idx++;
      }

      /* Check for flash memory ring-buffer wrap-around */
      if (PF.Massege_No >= (TotalFrameinflash + 1))
      {
        PF.FlashFull = 1;     // Mark flash full flag
        PF.Massege_No = 1;    // Wrap sequence count
        temp.SendCnt = 1;     // Reset transmit pointer
        PF.frame_idx = 0;
        PF.sector = 0;
        PF.sector_addr = 0;
        PF.flash_Add = 0;
      }
      
      xSemaphoreGive(flash_mutex); // Release SPI Flash mutex

      /* Persist updated pointers to dedicated power-fail sector */
      PF_mem_Write();
  }
}

/**
 * @brief Transfers a single byte over SPI1 in full-duplex mode.
 * Transmits `TxData` on MOSI while simultaneously shifting in a byte on MISO.
 * 
 * @param TxData Byte to transmit.
 * @return uint8_t Byte received from slave.
 */
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t retry = 0;

    /* Wait until SPI1 transmit buffer is empty (TXE) */
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        retry++;
        if(retry > 200) return 0;
    }
    SPI_I2S_SendData(SPI1, TxData);
    retry = 0;

    /* Wait until SPI1 receive buffer has data (RXNE) */
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
        retry++;
        if(retry > 200) return 0;
    }
    return SPI_I2S_ReceiveData(SPI1);
}

/**
 * @brief Initializes SPI1 peripheral and GPIO pins for Winbond Flash communication.
 * - PA5: SPI1_SCK (Alternate Function Push-Pull, 50MHz)
 * - PA7: SPI1_MOSI (Alternate Function Push-Pull, 50MHz)
 * - PA6: SPI1_MISO (Input Floating)
 * - PA4: W25Q_CS (General Purpose Push-Pull Output)
 * - SPI Mode: Master, Mode 3 (CPOL=High, CPHA=2Edge), Baud Prescaler 4.
 * - Clears Block Protect (BP) bits in flash status register to allow writes.
 * - Creates `flash_mutex` FreeRTOS mutex.
 */
void W25QXX_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure = {0};

    /* Enable clocks for GPIOA and SPI1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    /* PA5 (SCK), PA7 (MOSI) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA6 (MISO) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA4 (CS) */
    GPIO_InitStructure.GPIO_Pin = W25Q32_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    W25Q32_CS_HIGH(); // De-assert CS high initially

    /* SPI1 Master Configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;       // Clock idle high (Mode 3)
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;      // Data captured on second clock edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         // Software CS management
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // High speed clock
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;// MSB first transmission
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE); // Enable SPI1 peripheral
    
    // Clear Block Protect bits in Status Register so whole chip is writable
    W25Q32_Write_SR(0x00);

    /* Create FreeRTOS mutex for thread-safe flash access */
    if (flash_mutex == NULL) {
        flash_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief Reads the JEDEC Manufacturer and Device ID from the Winbond Flash chip.
 * Sends command 0x90 followed by 3 dummy address bytes (0x00, 0x00, 0x00),
 * then reads Manufacturer ID (0xEF) and Device ID.
 * @return uint16_t 16-bit Device ID (e.g. 0xEF19 for W25Q256).
 */
uint16_t W25Q32_ReadID(void)
{
    uint16_t Temp = 0;
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_ManufactDeviceID); // Send opcode 0x90
    SPI1_ReadWriteByte(0x00);                  // 24-bit dummy address byte 1
    SPI1_ReadWriteByte(0x00);                  // byte 2
    SPI1_ReadWriteByte(0x00);                  // byte 3
    Temp |= SPI1_ReadWriteByte(0xFF) << 8;     // Read Manufacturer ID (0xEF)
    Temp |= SPI1_ReadWriteByte(0xFF);          // Read Device ID (e.g. 0x19)
    W25Q32_CS_HIGH();
    return Temp;
}

/**
 * @brief Reads Status Register 1 from the flash chip.
 * Bit 0 is the BUSY flag:
 * - 1: Internal Write, Page Program, or Sector Erase cycle is in progress.
 * - 0: Device is ready for new commands.
 * @return uint8_t Status Register 1 byte.
 */
static uint8_t W25Q32_ReadSR(void)
{
    uint8_t byte = 0;
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_ReadStatusReg); // Opcode 0x05
    byte = SPI1_ReadWriteByte(0Xff);        // Read status byte
    W25Q32_CS_HIGH();
    return byte;
}

/**
 * @brief Blocks execution until the flash chip completes its internal erase or write operation.
 * Polls Status Register 1 until BUSY bit (bit 0) returns 0 or timeout expires.
 */
static void W25Q32_Wait_Busy(void)
{
    uint32_t timeout = 500000;
    while(((W25Q32_ReadSR() & 0x01) == 0x01) && --timeout > 0);
}

/**
 * @brief Sends the Write Enable (0x06) command to the flash chip.
 * Must be executed before every Page Program, Sector Erase, or Status Register write.
 */
static void W25Q32_Write_Enable(void)
{
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_WriteEnable); // Opcode 0x06
    W25Q32_CS_HIGH();
}

/**
 * @brief Writes a byte to Status Register 1.
 * Used during initialization to clear hardware write-protect (BP) bits.
 * @param sr Byte value to write to status register (0x00 enables writing to all sectors).
 */
static void W25Q32_Write_SR(uint8_t sr)
{
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(0x50); // Enable Write Status Register (Volatile SR Write Enable)
    W25Q32_CS_HIGH();
    
    W25Q32_Write_Enable();
    
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(0x01); // Opcode 0x01: Write Status Register
    SPI1_ReadWriteByte(sr);
    W25Q32_CS_HIGH();
    W25Q32_Wait_Busy();
}

/**
 * @brief Reads arbitrary data bytes from flash using the Fast Read (0x0B) command.
 * 
 * Sequence:
 * 1. Pull CS LOW.
 * 2. Send opcode 0x0B.
 * 3. Send 24-bit byte address (MSB first).
 * 4. Send 1 dummy byte (required by W25Q Fast Read protocol).
 * 5. Clock in data bytes.
 * 6. Pull CS HIGH.
 * 
 * @param pBuffer Pointer to destination buffer.
 * @param ReadAddr 24-bit starting byte address.
 * @param NumByteToRead Number of bytes to read.
 */
void SPI_Flash_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint16_t i;
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_FastReadData);              // Opcode 0x0B
    SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 16));     // Address [23..16]
    SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 8));      // Address [15..8]
    SPI1_ReadWriteByte((uint8_t)ReadAddr);              // Address [7..0]
    
    // Fast Read (0x0B) requires 1 dummy byte!
    SPI1_ReadWriteByte(0xFF); 

    /* Read requested bytes */
    for(i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = SPI1_ReadWriteByte(0XFF);
    }
    W25Q32_CS_HIGH();
}

/**
 * @brief Programs up to 256 bytes within a single 256-byte flash page.
 * 
 * Winbond Flash requires Page Program (0x02) to not cross a 256-byte page boundary.
 * 
 * @param pBuffer Pointer to source data buffer.
 * @param WriteAddr Target byte address.
 * @param NumByteToWrite Number of bytes to program (1 to 256).
 */
void W25Q32_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;
    W25Q32_Write_Enable();                              // Enable write latch
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_PageProgram);               // Opcode 0x02
    SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 16));    // Address [23..16]
    SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 8));     // Address [15..8]
    SPI1_ReadWriteByte((uint8_t)WriteAddr);             // Address [7..0]
    for(i = 0; i < NumByteToWrite; i++)
    {
        SPI1_ReadWriteByte(pBuffer[i]);
    }
    W25Q32_CS_HIGH();
    W25Q32_Wait_Busy(); // Wait for internal page program cycle to complete
}

/**
 * @brief Writes arbitrary length of data to flash, automatically splitting across 256-byte page boundaries.
 * 
 * Calculates remaining space in the current page, writes up to that boundary using W25Q32_Write_Page(),
 * and loops until all bytes are written.
 * 
 * @param pBuffer Pointer to source data.
 * @param WriteAddr Absolute flash write address.
 * @param NumByteToWrite Total number of bytes to write.
 */
void SPI_Flash_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256; // Calculate bytes remaining in starting page
    if(NumByteToWrite <= pageremain) pageremain = NumByteToWrite;
    while(1)
    {
        W25Q32_Write_Page(pBuffer, WriteAddr, pageremain);
        if(NumByteToWrite == pageremain) break; // Finished writing all data
        else
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;
            NumByteToWrite -= pageremain;
            if(NumByteToWrite > 256) pageremain = 256; // Full page
            else pageremain = NumByteToWrite;          // Final partial page
        }
    }
}

/**
 * @brief Erases a 4096-byte (4KB) sector in SPI Flash.
 * All bytes in the sector are reset to 0xFF.
 * 
 * @param Dst_Addr Sector index (0 to 8191). Multiplied by 4096 to get starting byte address.
 */
void SPI_Flash_Erase_Sector(uint32_t Dst_Addr)
{
    Dst_Addr *= 4096; // Convert sector index to byte address
    W25Q32_Write_Enable();
    W25Q32_Wait_Busy();
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25X_SectorErase);             // Opcode 0x20
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr) >> 16));  // Address [23..16]
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr) >> 8));   // Address [15..8]
    SPI1_ReadWriteByte((uint8_t)Dst_Addr);           // Address [7..0]
    W25Q32_CS_HIGH();
    W25Q32_Wait_Busy(); // Wait for sector erase operation (typically 45-400ms)
}
