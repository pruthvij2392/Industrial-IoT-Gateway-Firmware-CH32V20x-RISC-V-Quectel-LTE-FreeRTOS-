#include "ds3231.h"

/**
 * @brief Initializes the MCU I2C1 peripheral and its corresponding GPIO pins.
 * - PB6 configured as I2C1_SCL (Alternate Function Open-Drain, 50MHz)
 * - PB7 configured as I2C1_SDA (Alternate Function Open-Drain, 50MHz)
 * - Configures I2C1 in Master mode, 100 kHz standard clock speed, 7-bit addressing.
 */
void DS3231_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitStructure = {0};

    /* Enable peripheral clock for GPIOB, AFIO (Pin remapping/alternate function), and I2C1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* Configure PB6 (SCL) and PB7 (SDA) as Open-Drain Output for I2C bus compliance */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure I2C1 parameters */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000; // 100 kHz standard I2C speed

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE); // Enable I2C1 peripheral
}

/**
 * @brief Maximum loop counter used as a timeout guard for I2C flag checks.
 * Prevents the CPU from permanently hanging if the slave does not respond or the bus is stuck.
 */
#define I2C_TIMEOUT  10000

/**
 * @brief Resets and recovers the I2C1 peripheral when a bus timeout or lockup occurs.
 * - Sends an I2C STOP condition.
 * - Toggles I2C peripheral software reset (SWRST).
 * - Re-runs DS3231_Init() to reinitialize the peripheral state cleanly.
 */
static void DS3231_I2C_Reset(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    DS3231_Init();
}

/**
 * @brief Writes a single byte of data to a specific register inside the DS3231 RTC.
 * 
 * Communication sequence:
 * 1. Wait until I2C bus is not busy (FLAG_BUSY == RESET).
 * 2. Generate START condition and wait for MASTER_MODE_SELECT.
 * 3. Send 7-bit DS3231 slave address with write bit (0xD0) and wait for ADDR match.
 * 4. Send register address byte and wait for BYTE_TRANSMITTED.
 * 5. Send data byte to write into that register and wait for BYTE_TRANSMITTED.
 * 6. Generate STOP condition.
 * 
 * @param reg Register address in DS3231 (e.g. 0x00 to 0x12).
 * @param data Data byte to be written.
 */
static void DS3231_WriteReg(uint8_t reg, uint8_t data)
{
    uint32_t to;

    /* 1. Wait until bus is idle */
    to = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return; }

    /* 2. Generate START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return; }

    /* 3. Send DS3231 I2C address for writing */
    I2C_Send7bitAddress(I2C1, DS3231_ADDRESS, I2C_Direction_Transmitter);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return; }

    /* 4. Send target internal register address */
    I2C_SendData(I2C1, reg);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return; }

    /* 5. Send data byte */
    I2C_SendData(I2C1, data);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return; }

    /* 6. Generate STOP condition to complete write */
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/**
 * @brief Reads a single byte of data from a specified register in the DS3231 RTC.
 * 
 * Communication sequence:
 * 1. Wait until bus is idle.
 * 2. Generate START condition and select Master Transmitter mode.
 * 3. Send DS3231 write address (0xD0) followed by target register address.
 * 4. Generate Repeated START condition.
 * 5. Send DS3231 read address (0xD1) and wait for Master Receiver mode.
 * 6. Disable ACK (NACK) and generate STOP condition before reading byte (single-byte read protocol).
 * 7. Read received byte from I2C data register and re-enable ACK for future transfers.
 * 
 * @param reg Register address in DS3231 to read from.
 * @return uint8_t Register value received from DS3231 (or 0 if timeout occurs).
 */
static uint8_t DS3231_ReadReg(uint8_t reg)
{
    uint8_t temp = 0;
    uint32_t to;

    /* 1. Wait until bus is idle */
    to = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* 2. Generate START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* 3. Send address with write bit to set register pointer */
    I2C_Send7bitAddress(I2C1, DS3231_ADDRESS, I2C_Direction_Transmitter);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* 4. Send register address */
    I2C_SendData(I2C1, reg);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }
    
    /* 5. Repeated START for reading */
    I2C_GenerateSTART(I2C1, ENABLE);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* 6. Send address with read direction */
    I2C_Send7bitAddress(I2C1, DS3231_ADDRESS, I2C_Direction_Receiver);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* For single byte read: disable ACK and trigger STOP before reading data */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);

    to = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) && --to > 0);
    if (to == 0) { DS3231_I2C_Reset(); return 0; }

    /* Read byte from data register */
    temp = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, ENABLE); // Restore ACK for future operations
    return temp;
}

/**
 * @brief Converts a Binary-Coded Decimal (BCD) byte to standard binary decimal.
 * DS3231 stores time in BCD format where high nibble is tens digit and low nibble is units digit.
 * Example: 0x25 -> (2 * 10) + 5 = 25.
 * @param bcd BCD encoded byte.
 * @return uint8_t Standard decimal integer.
 */
static uint8_t BCD2DEC(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

/**
 * @brief Converts a standard binary decimal integer to Binary-Coded Decimal (BCD).
 * Used when writing user/network time into DS3231 registers.
 * Example: 25 -> ((25 / 10) << 4) | (25 % 10) = 0x25.
 * @param dec Standard decimal integer (0 to 99).
 * @return uint8_t BCD encoded byte.
 */
static uint8_t DEC2BCD(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

/**
 * @brief Reads date and time from DS3231 registers and populates the Run_Send_t structure.
 * 
 * DS3231 Register Map:
 * - 0x00: Seconds (00-59 BCD)
 * - 0x01: Minutes (00-59 BCD)
 * - 0x02: Hours (bit6: 12/24hr select. In 24hr mode, bits 0-5 hold hours 00-23, masked with 0x3F)
 * - 0x03: Day of week (1-7)
 * - 0x04: Date of month (01-31 BCD)
 * - 0x05: Month and Century (bits 0-4 hold month 01-12, masked with 0x7F to ignore century bit)
 * - 0x06: Year (00-99 BCD)
 * 
 * @param time Pointer to Run_Send_t destination structure.
 */
void ds3231_get_time(Run_Send_t *time)
{
    time->second = BCD2DEC(DS3231_ReadReg(0x00));
    time->minute = BCD2DEC(DS3231_ReadReg(0x01));
    time->hour   = BCD2DEC(DS3231_ReadReg(0x02) & 0x3F);
    time->day    = BCD2DEC(DS3231_ReadReg(0x03));
    time->date   = BCD2DEC(DS3231_ReadReg(0x04));
    time->month  = BCD2DEC(DS3231_ReadReg(0x05) & 0x7F);
    time->year   = BCD2DEC(DS3231_ReadReg(0x06));
}

/**
 * @brief Writes new date and time values from Run_Send_t into the DS3231 registers.
 * Converts each decimal field to BCD before writing to registers 0x00 through 0x06.
 * @param time Pointer to Run_Send_t source structure containing new time.
 */
void ds3231_set_time(Run_Send_t *time)
{
    DS3231_WriteReg(0x00, DEC2BCD(time->second));
    DS3231_WriteReg(0x01, DEC2BCD(time->minute));
    DS3231_WriteReg(0x02, DEC2BCD(time->hour));
    DS3231_WriteReg(0x03, DEC2BCD(time->day));
    DS3231_WriteReg(0x04, DEC2BCD(time->date));
    DS3231_WriteReg(0x05, DEC2BCD(time->month));
    DS3231_WriteReg(0x06, DEC2BCD(time->year));
}

/**
 * @brief Parses an ASCII time string (such as Quectel cellular AT+QLTS network timestamp)
 * and programs it into the DS3231 RTC.
 * 
 * Expected ASCII String layout:
 * StrTime: "YY/MM/DD,HH:MM:SS" (or similar formatted string where digits are at specific offsets):
 * - StrTime[0..1]: Year (e.g. "26")
 * - StrTime[3..4]: Month (e.g. "10")
 * - StrTime[6..7]: Date (e.g. "28")
 * - StrTime[9..10]: Hour (e.g. "15")
 * - StrTime[12..13]: Minute (e.g. "30")
 * - StrTime[15..16]: Second (e.g. "00")
 * 
 * @param StrTime Pointer to ASCII string.
 */
void RTC_WrtFromStr(uint8_t *StrTime)
{
  Send.second = (uint8_t)(((StrTime[15] - '0') * 10) + (StrTime[16] - '0'));
  Send.minute = (uint8_t)(((StrTime[12] - '0') * 10) + (StrTime[13] - '0'));
  Send.hour   = (uint8_t)(((StrTime[9] - '0') * 10) + (StrTime[10] - '0'));
  Send.date   = (uint8_t)(((StrTime[6] - '0') * 10) + (StrTime[7] - '0'));
  Send.month  = (uint8_t)(((StrTime[3] - '0') * 10) + (StrTime[4] - '0'));
  Send.year   = (uint8_t)(((StrTime[0] - '0') * 10) + (StrTime[1] - '0'));
  ds3231_set_time(&Send);
}

/**
 * @brief Enables the 1 Hz square wave output on the DS3231 INT/SQW pin.
 * Writes 0x00 to Control Register 0x0E:
 * - Bit 7 (EOSC = 0): Oscillator enabled
 * - Bit 6 (BBSQW = 0): Battery-backed square-wave disable
 * - Bit 5 (CONV = 0): No forced temperature conversion
 * - Bits 4..3 (RS2=0, RS1=0): Square-wave frequency set to 1 Hz
 * - Bit 2 (INTCN = 0): Output Square Wave instead of Alarm interrupts
 * - Bits 1..0 (A2IE=0, A1IE=0): Alarm interrupts disabled
 */
void DS3231_Enable1Hz(void)
{
    DS3231_WriteReg(0x0E, 0x00);
}

/**
 * @brief External Interrupt Line 0 Handler (EXTI0).
 * Triggered every 1 second by the falling edge of the DS3231 1Hz SQW pin connected to PA0.
 * Sets the global timing flag `temp.g_sqw_flag = 1` for `iot_task` to handle 1-second system tasks.
 */
void EXTI0_IRQHandler(void) __attribute__((interrupt()));
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        temp.g_sqw_flag = 1; // Signal 1-second tick to FreeRTOS iot_task
        EXTI_ClearITPendingBit(EXTI_Line0); // Clear hardware interrupt pending bit
    }
}

/**
 * @brief Initializes PA0 as an external interrupt input (EXTI0) to capture 1 Hz pulses from DS3231.
 * - Configures PA0 as Input with internal Pull-Up (IPU).
 * - Routes GPIOA Pin 0 to EXTI Line 0.
 * - Sets falling-edge trigger (active low pulse from DS3231 open-drain SQW output).
 * - Enables EXTI0_IRQn in NVIC with priority (Preemption: 1, SubPriority: 2).
 * - Calls DS3231_Enable1Hz() to begin generating the 1 Hz square wave.
 */
void DS3231_INT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* Enable clock for AFIO (Alternate Function I/O) and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure PA0 as Input with Pull-Up for RTC INT/SQW signal */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect EXTI Line 0 to GPIOA Pin 0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    /* Configure EXTI Line 0 for falling edge trigger */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Configure NVIC for EXTI0 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Turn on 1 Hz square wave output from DS3231 */
    DS3231_Enable1Hz();
}
