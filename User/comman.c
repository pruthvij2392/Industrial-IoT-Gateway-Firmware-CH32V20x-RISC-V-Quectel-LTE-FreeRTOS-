#include "comman.h"
#include "rs485.h"
#include "w25qxx.h"

/* ==============================================================================
 * GLOBAL VARIABLE ALLOCATIONS
 * ============================================================================== */

/**
 * @brief Volatile runtime state structure.
 * Tracks SendCnt sequence, DS3231 1Hz interrupt flag, and boot power status.
 */
tempery temp = {0};

/**
 * @brief System persistent configuration memory.
 * Stores MQTT broker, serial number, device ID, sample time, and Modbus mapping table.
 */
memory first = {0};

/**
 * @brief Power-fail (PF) persistent memory.
 * Stores flash write addresses, frame indices, message count, and wrap-around flags.
 */
powerfailmem PF = {0};

/**
 * @brief Live telemetry structure continuously populated from sensors, RTC, and GNSS.
 */
Run_Send_t Run = {0};

/**
 * @brief Staged telemetry frame snapshot copied from `Run` when sample interval triggers.
 * Used for flash writing and cloud transmission.
 */
Run_Send_t Send = {0};

/**
 * @brief Magic identification number (30) stored in flash memory headers.
 * Used to verify if external SPI Flash sectors have been previously formatted.
 */
uint8_t magic_no = 30;

/**
 * @brief Cellular SIM Operation Mode:
 * - 1: Single SIM mode (fixed slot)
 * - 2: Dual SIM auto-failover mode
 */
uint8_t SIM_MODE = 1;

/**
 * @brief Resets power-fail indices, message counters, and offline ring buffer addresses.
 * Called during system factory reset or via the "reset" JSON command.
 * Resets message counter to 1, addresses to 0, and saves updated state to SPI Flash.
 */
void Device_Reset(void) {
  PF.Massege_No = 1;      // Start message count from 1
  PF.flash_Add = 0;       // Reset flash write pointer to address 0
  PF.frame_idx = 0;       // Reset frame index in sector
  PF.sector = 0;          // Reset sector to 0
  PF.sector_addr = 0;     // Reset sector base address to 0
  PF.Rd_Frame_idx = 0;    // Reset total recorded frame count
  temp.SendCnt = 1;       // Reset transmission pointer to record 1
  PF.FlashFull = 0;       // Clear flash full/rollover flag
  PF.FileNo = 1;          // Set file number to default
  PF.Verify_PF = magic_no;// Set validation magic number (30)
  PF_mem_Write();         // Persist reset state to external SPI Flash
}

/**
 * @brief Writes the `first` system configuration structure to external SPI Flash.
 * - Guards SPI bus access using `flash_mutex` to prevent task collisions.
 * - Erases sector `First_MEM_Sector`.
 * - Delays 5ms for erase cycle completion.
 * - Writes `first` structure bytes without check.
 * - Releases mutex.
 */
void Set_mem_Write(void) {
  uint32_t address;
  address = First_MEM_Sector * 4096; // Calculate 4KB sector starting byte address
  if (flash_mutex != NULL) {
    xSemaphoreTake(flash_mutex, portMAX_DELAY); // Acquire exclusive SPI Flash access
  }
  SPI_Flash_Erase_Sector(First_MEM_Sector);     // Erase 4KB configuration sector
  vTaskDelay(pdMS_TO_TICKS(5));                 // Brief delay for flash write recovery
  SPI_Flash_Write_NoCheck((uint8_t *)&first, address, sizeof(first)); // Write config block
  if (flash_mutex != NULL) {
    xSemaphoreGive(flash_mutex);                // Release SPI Flash mutex
  }
}

/**
 * @brief Reads system configuration from SPI Flash into the `first` structure.
 * Checks the validation byte (`first.Verify1`). If not equal to `magic_no` (30)
 * (indicating blank/uninitialized flash), populates factory default settings and writes to flash.
 */
void Set_mem_Read(void) {
  uint32_t AddSec;
  AddSec = First_MEM_Sector * 4096;
  if (flash_mutex != NULL) {
    xSemaphoreTake(flash_mutex, portMAX_DELAY); // Acquire exclusive SPI Flash access
  }
  SPI_Flash_Read((uint8_t *)&first, AddSec, sizeof(first)); // Read config structure from flash
  if (flash_mutex != NULL) {
    xSemaphoreGive(flash_mutex);                // Release SPI Flash mutex
  }

  /* Check if flash has not been formatted or magic number is invalid */
  if (first.Verify1 != magic_no) {
    memset(&first, 0, sizeof(first));

    /* Initialize Factory Default Settings */
    first.Verify1 = magic_no;          // Set format verification flag (30)
    first.Sample_Time = 2;             // Default telemetry sampling: 2 seconds
    first.MQTTPort = 1883;             // Default standard MQTT port

    strcpy(first.Serial_no, "26100001");              // Factory Default Serial Number
    strcpy(first.Device_ID, "GTW001");                // Factory Default Device ID
    strcpy(first.MQTTServer, "mqtt.swasemi.in");      // Default MQTT Broker URL
    strcpy(first.MQTTUsername, "mqtt_swasemi");       // Default MQTT Username
    strcpy(first.MQTTPassword, "mqtt@Swasemi@11cr");   // Default MQTT Password

    first.Parity = _none;              // RS-485 Parity: None
    first.Stop_bit = 1;                // RS-485 Stop bits: 1
    first.Baud_Rate = _9600;           // RS-485 Baud rate: 9600 bps
    first.No_Of_Slave = 2;             // Default active Modbus slaves: 2

    /* Initialize all 16 Modbus slave / register slots to default values */
    for (uint8_t y = 0; y < 16; y++) {
      first.SlaveID[y] = y + 1;         // Slave IDs: 1, 2, 3...
      first.FunctionID[y] = 3;          // Function Code: 3 (Read Holding Registers)
      first.ModbusAdd[y] = 0;           // Start register address: 0
      first.BytetoRead[y] = 2;          // Number of registers to read: 2
      first.DataType[y] = TYPE_INT16;   // Data type: 16-bit signed integer
      first.Decimal[y] = 0;             // Decimal places: 0
      first.Endian[y] = BIG_Endian;     // Endianness: Big Endian (ABCD)
      first.DisValAdd[y] = y * 100;     // Display value offset index
    }

    /* Save factory defaults into SPI Flash */
    Set_mem_Write();
  }
}

/**
 * @brief Writes the Power-Fail memory (`PF`) structure into dedicated sector `PF_MEM_Sector`.
 * Preserves the current write address, frame indices, and sequence counts across power cycles.
 */
void PF_mem_Write(void) {
  uint32_t address;
  address = PF_MEM_Sector * 4096; // Sector address
  if (flash_mutex != NULL) {
    xSemaphoreTake(flash_mutex, portMAX_DELAY); // Acquire exclusive SPI Flash access
  }
  SPI_Flash_Erase_Sector(PF_MEM_Sector);        // Erase 4KB power-fail sector
  vTaskDelay(pdMS_TO_TICKS(5));                 // Delay for sector erase completion
  SPI_Flash_Write_NoCheck((uint8_t *)&PF, address, sizeof(PF)); // Write PF structure
  if (flash_mutex != NULL) {
    xSemaphoreGive(flash_mutex);                // Release SPI Flash mutex
  }
}

/**
 * @brief Reads the Power-Fail memory (`PF`) structure from SPI Flash.
 * If the magic number `PF.Verify_PF` is not equal to `magic_no` (30), initializes
 * the power-fail memory to starting defaults and saves to flash.
 */
void PF_mem_Read(void) {
  uint32_t AddSec;
  AddSec = PF_MEM_Sector * 4096;
  if (flash_mutex != NULL) {
    xSemaphoreTake(flash_mutex, portMAX_DELAY); // Acquire exclusive SPI Flash access
  }
  SPI_Flash_Read((uint8_t *)&PF, AddSec, sizeof(PF)); // Read PF structure from flash
  if (flash_mutex != NULL) {
    xSemaphoreGive(flash_mutex);                // Release SPI Flash mutex
  }

  /* Check if power-fail sector has been formatted */
  if (PF.Verify_PF != magic_no) {
    memset(&PF, 0, sizeof(PF));

    PF.Verify_PF = magic_no; // Validation magic number (30)
    PF.FlashFull = 0;        // Ring buffer not full
    PF.frame_idx = 0;        // Start at frame 0
    PF.sector = 0;           // Start at sector 0
    PF.sector_addr = 0;      // Start at byte 0
    PF.flash_Add = 0;        // Start at address 0
    PF.Rd_Frame_idx = 0;     // 0 recorded frames
    PF.Massege_No = 1;       // Message sequence starts at 1
    PF.FileNo = 1;           // Default file index

    PF_mem_Write();          // Save initial state to flash
  }
}

/**
 * @brief Initializes all status indicator LEDs and peripheral power GPIO pins.
 * - PA1: NW_LED (Cellular Network status)
 * - PA15: GPS_LED (GNSS fix status)
 * - PB0: DATA_LED (MQTT publish / data activity)
 * - PB1: PWR_LED (System Power indicator)
 * - PB3: Modem PWRKEY control pin
 * - PB4: Modem POWER enable pin
 */
void gpio_pin_init(void) {
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  /* Enable peripheral clocks for GPIOA and GPIOB */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure PA1 (NW_LED) and PA15 (GPS_LED) as Push-Pull Outputs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PB0 (DATA_LED), PB1 (PWR_LED), PB3 (Modem PWRKEY), and PB4 (Modem POWER) as Push-Pull Outputs */
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Set initial GPIO states:
   * - Turn OFF activity LEDs (PB0, PA15, PA1)
   * - Turn ON PWR_LED (PB1) immediately upon system power
   * - Keep modem control pins (PB3, PB4) LOW until power sequence begins
   */
  GPIO_ResetBits(GPIOB, GPIO_Pin_0); // DATA_LED OFF
  GPIO_ResetBits(GPIOA, GPIO_Pin_15);// GPS_LED OFF
  GPIO_ResetBits(GPIOA, GPIO_Pin_1); // NW_LED OFF
  GPIO_SetBits(GPIOB, GPIO_Pin_1);   // PWR_LED ON

  GPIO_ResetBits(GPIOB, GPIO_Pin_3); // Modem PWRKEY LOW
  GPIO_ResetBits(GPIOB, GPIO_Pin_4); // Modem POWER LOW
}
