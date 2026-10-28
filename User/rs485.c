#include "rs485.h"
#include "FreeRTOS.h"
#include "task.h"
#include "comman.h"

/* Global instance of Modbus RTU state structure */
modbusRTU modbus = {0};

/* Local transmission pointers and synchronization flags */
static uint8_t *tx_ptr;                     /**< Pointer to next byte to transmit via USART1 TX interrupt */
static volatile uint16_t tx_len = 0;        /**< Remaining bytes left to transmit */
static volatile bool g_uart_tx_complete = true; /**< Flag indicating RS-485 transceiver has completed TX */

void Modbus_LCDPrint(void);
void Modbus_Data_Tx(uint8_t slave, uint8_t FC, uint16_t Add, uint16_t BtR);
void Modbus_Parse(void * dest, uint8_t * src, MBDataType_t type, ModbusEndian_t endian);
void Modbus_Uart_Transmit(uint8_t *send_str, uint16_t send_len);
uint16_t ModbusCRC16(uint8_t *Buffer, uint16_t Length, uint8_t Crc_Check);

/**
 * @brief Prepares and transmits a Modbus RTU read query for slave device index `DeviceNo`.
 * - Tracks slave error count: if a slave fails to respond for 3 consecutive polls,
 *   its holding register buffer is cleared to 0 and its display value is set to "0".
 * - Dispatches request via Modbus_Data_Tx().
 * @param DeviceNo Slave slot index (0 to No_Of_Slave - 1).
 */
void Modbus_Send(uint8_t DeviceNo)
{
    /* Increment communication error counter for this slave */
    modbus.Error_Cnt[DeviceNo]++;

    /* Timeout threshold: if slave fails 3 times consecutively, mark values as offline ("0") */
    if(modbus.Error_Cnt[DeviceNo] >= 3){
        modbus.Error_Cnt[DeviceNo] = 0;
        memset(modbus.HoldReg[DeviceNo], 0, sizeof(modbus.HoldReg[DeviceNo]));
        strcpy(Run.Modbus_Val[DeviceNo], "0");
    }

    modbus.Dno = DeviceNo; // Record active device index

    /* Build and transmit standard Modbus RTU frame */
    Modbus_Data_Tx(first.SlaveID[DeviceNo], first.FunctionID[DeviceNo], first.ModbusAdd[DeviceNo], first.BytetoRead[DeviceNo]);
}

/**
 * @brief Constructs an 8-byte standard Modbus RTU Read Request frame and transmits it.
 * 
 * Frame Format:
 * [Byte 0]: Slave ID (1-255)
 * [Byte 1]: Function Code (0x03 = Read Holding Registers, 0x04 = Read Input Registers)
 * [Byte 2]: Start Address High Byte
 * [Byte 3]: Start Address Low Byte
 * [Byte 4]: Number of Registers High Byte
 * [Byte 5]: Number of Registers Low Byte
 * [Byte 6]: CRC-16 Low Byte
 * [Byte 7]: CRC-16 High Byte
 * 
 * @param slave Slave Address (1 to 255)
 * @param FC Function Code (3 or 4)
 * @param Add Register Start Address (0 to 65535)
 * @param BtR Quantity of 16-bit Registers to read (1 to 30)
 */
void Modbus_Data_Tx(uint8_t slave, uint8_t FC, uint16_t Add, uint16_t BtR)
{
    modbus.Tx_Buff[0]  = slave;
    modbus.Tx_Buff[1]  = FC;
    modbus.Tx_Buff[2]  = (uint8_t)((Add >> 8) & 0xff);
    modbus.Tx_Buff[3]  = (uint8_t)(Add & 0xff);
    modbus.Tx_Buff[4]  = (uint8_t)((BtR >> 8) & 0xff);
    modbus.Tx_Buff[5]  = (uint8_t)(BtR  & 0xff);

    /* Calculate 16-bit CRC over first 6 bytes and append CRC_Low / CRC_High */
    ModbusCRC16(modbus.Tx_Buff, 6, CRC_GET);

    /* Transmit the 8-byte packet over RS-485 */
    Modbus_Uart_Transmit(modbus.Tx_Buff, 8);
}

/**
 * @brief Validates and processes a completed Modbus RTU response packet received in `Rx_Buff`.
 * 
 * Verification Steps:
 * 1. Checks that response Slave ID matches active polled Slave ID.
 * 2. Computes and verifies CRC-16 across the payload.
 * 3. Copies register data bytes into `modbus.HoldReg[modbus.Dno]`.
 * 4. Calls Modbus_LCDPrint() to unpack and format the value into ASCII.
 * 5. Clears the error counter on successful reception.
 */
void Modbus_Data_Rx()
{
    if(modbus.Rx_Buff[0] == first.SlaveID[modbus.Dno])
    {
        /* Modbus Response packet length = Byte_Count (at index 2) + 3 header bytes + 2 CRC bytes */
        if(ModbusCRC16(modbus.Rx_Buff, (modbus.Rx_Buff[2] + 3), CRC_CHECK) == 1)
        {
            /* Copy data payload into slave holding register memory table */
            memcpy(&modbus.HoldReg[modbus.Dno][0], &modbus.Rx_Buff[3], modbus.Rx_Buff[2]);
            modbus.buffCnt = modbus.Dno;

            /* Parse and format sensor values */
            Modbus_LCDPrint();
        }
        /* Reset error counter since slave responded successfully */
        modbus.Error_Cnt[modbus.Dno] = 0;
    }
}

/**
 * @brief Power of 10 lookup table for fast integer decimal scaling (10^0 through 10^6).
 */
static const uint32_t p10[7] = {1, 10, 100, 1000, 10000, 100000, 1000000};

/**
 * @brief Fast, re-entrant floating point to ASCII formatter.
 * Avoids the massive code size, stack overhead, and heap dependencies of standard sprintf("%f").
 * 
 * @param buf Output string buffer.
 * @param val Floating point number to format.
 * @param dec Number of decimal places to include (0 to 6).
 */
static void sprint_float(char *buf, float val, uint8_t dec)
{
    char *p = buf;
    if (val < 0.0f) {
        *p++ = '-';
        val = -val;
    }
    if (dec > 6) dec = 6;
    uint32_t m = p10[dec];
    uint32_t ip = (uint32_t)val; // Integer part
    uint32_t fp = (uint32_t)((val - (float)ip) * (float)m + 0.5f); // Scaled fractional part with rounding
    if (fp >= m) {
        fp -= m;
        ip++;
    }
    if (ip == 0 && fp == 0) p = buf; // Avoid printing negative zero "-0.00"
    if (dec == 0) {
        sprintf(p, "%lu", (unsigned long)ip);
    } else {
        sprintf(p, "%lu.%0*lu", (unsigned long)ip, (int)dec, (unsigned long)fp);
    }
}

/**
 * @brief Decodes raw register bytes into numerical values and formats ASCII string for telemetry.
 * 
 * Based on the configured `DataType[buffCnt]`:
 * - Case 0: TYPE_UINT16 (Unsigned 16-bit integer)
 * - Case 1: TYPE_INT16  (Signed 16-bit integer)
 * - Case 2: TYPE_UINT32 (Unsigned 32-bit integer)
 * - Case 3: TYPE_INT32  (Signed 32-bit integer)
 * - Case 4: TYPE_FLOAT  (32-bit IEEE 754 Floating point)
 * 
 * Applies decimal point division and writes formatted string into `Run.Modbus_Val[buffCnt]`.
 */
void Modbus_LCDPrint()
{
    float Dis_Var = 0.0f;
    uint8_t BuffDis[16] = {0};
    uint8_t d = first.Decimal[modbus.buffCnt];
    if (d > 6) d = 6;

    switch(first.DataType[modbus.buffCnt])
    {
        case 0: // TYPE_UINT16
            Modbus_Parse(&modbus.U16_Value[modbus.buffCnt], &(*modbus.HoldAdd[modbus.buffCnt]), TYPE_UINT16, first.Endian[modbus.buffCnt]);
            if (d == 0) {
                snprintf((char *)BuffDis, sizeof(BuffDis), "%u", modbus.U16_Value[modbus.buffCnt]);
            } else {
                Dis_Var = (float)modbus.U16_Value[modbus.buffCnt] / (float)p10[d];
                sprint_float((char *)BuffDis, Dis_Var, d);
            }
            break;

        case 1: // TYPE_INT16
            Modbus_Parse(&modbus.I16_Value[modbus.buffCnt], &(*modbus.HoldAdd[modbus.buffCnt]), TYPE_INT16, first.Endian[modbus.buffCnt]);
            if (d == 0) {
                snprintf((char *)BuffDis, sizeof(BuffDis), "%d", modbus.I16_Value[modbus.buffCnt]);
            } else {
                Dis_Var = (float)modbus.I16_Value[modbus.buffCnt] / (float)p10[d];
                sprint_float((char *)BuffDis, Dis_Var, d);
            }
            break;

        case 2: // TYPE_UINT32
            Modbus_Parse(&modbus.U32_Value[modbus.buffCnt], &(*modbus.HoldAdd[modbus.buffCnt]), TYPE_UINT32, first.Endian[modbus.buffCnt]);
            if (d == 0) {
                snprintf((char *)BuffDis, sizeof(BuffDis), "%lu", (unsigned long)modbus.U32_Value[modbus.buffCnt]);
            } else {
                Dis_Var = (float)modbus.U32_Value[modbus.buffCnt] / (float)p10[d];
                sprint_float((char *)BuffDis, Dis_Var, d);
            }
            break;

        case 3: // TYPE_INT32
            Modbus_Parse(&modbus.I32_Value[modbus.buffCnt], &(*modbus.HoldAdd[modbus.buffCnt]), TYPE_INT32, first.Endian[modbus.buffCnt]);
            if (d == 0) {
                snprintf((char *)BuffDis, sizeof(BuffDis), "%ld", (long)modbus.I32_Value[modbus.buffCnt]);
            } else {
                Dis_Var = (float)modbus.I32_Value[modbus.buffCnt] / (float)p10[d];
                sprint_float((char *)BuffDis, Dis_Var, d);
            }
            break;

        case 4: // TYPE_FLOAT
            Modbus_Parse(&modbus.F32_Value[modbus.buffCnt], &(*modbus.HoldAdd[modbus.buffCnt]), TYPE_FLOAT, first.Endian[modbus.buffCnt]);
            sprint_float((char *)BuffDis, modbus.F32_Value[modbus.buffCnt], d);
            break;

        default:
            break;
    }

    /* Store formatted ASCII result into active telemetry frame */
    memset(&Run.Modbus_Val[modbus.buffCnt], 0, sizeof(Run.Modbus_Val[modbus.buffCnt]));
    if (strlen((char *)BuffDis) == 0) {
        strcpy(Run.Modbus_Val[modbus.buffCnt], "0");
    } else {
        strncpy(Run.Modbus_Val[modbus.buffCnt], (char *)BuffDis, sizeof(Run.Modbus_Val[modbus.buffCnt]) - 1);
    }

    /* Advance buffer counter */
    ++modbus.buffCnt;
    if(modbus.buffCnt >= 16) {
        modbus.buffCnt = 0;
    }
}

/**
 * @brief Unpacks multi-byte register data according to the configured endianness / byte order.
 * 
 * Re-orders bytes into host architecture (Little-Endian RISC-V):
 * - For 16-bit values (2 bytes): handles Big Endian (AB -> BA) vs Little Endian (AB -> AB).
 * - For 32-bit values (4 bytes):
 *   - BIG_Endian (ABCD): MSW first, MSB first -> re-ordered to DCBA.
 *   - LITTLE_Endian (DCBA): LSW first, LSB first -> copied as DCBA.
 *   - BIG_ENDIAN_ByteSwap (BADC): Word swapped.
 *   - LITTLE_ENDIAN_ByteSwap (CDAB): Mid-Big endian.
 * 
 * @param dest Pointer to destination variable (U16, I16, U32, I32, or float).
 * @param src Pointer to raw bytes in Modbus holding register table.
 * @param type Data type category (16-bit vs 32-bit).
 * @param endian Endianness model enum.
 */
void Modbus_Parse(void * dest, uint8_t * src, MBDataType_t type, ModbusEndian_t endian) {
    uint8_t tempBuff[4] = {0};

    if (type == TYPE_UINT16 || type == TYPE_INT16) {
        /* 16-bit handling (2 bytes) */
        if (endian == BIG_Endian || endian == LITTLE_ENDIAN_ByteSwap) {
            // Big Endian Byte Order (swap bytes for Little-Endian MCU)
            tempBuff[0] = src[1];
            tempBuff[1] = src[0];
        }
        else {
            // Little Endian Byte Order
            tempBuff[0] = src[0];
            tempBuff[1] = src[1];
        }
        memcpy(dest, tempBuff, 2);
    }
    else {
        /* 32-bit handling (4 bytes) */
        switch(endian) {
            case BIG_Endian: // Big Endian (ABCD) -> Convert to host DCBA
                tempBuff[0] = src[3]; tempBuff[1] = src[2]; tempBuff[2] = src[1]; tempBuff[3] = src[0]; break;
            case LITTLE_Endian: // Already Little Endian (DCBA)
                memcpy(tempBuff, src, 4); break;
            case BIG_ENDIAN_ByteSwap: // (BADC) -> Convert to host DCBA
                tempBuff[0] = src[2]; tempBuff[1] = src[3]; tempBuff[2] = src[0]; tempBuff[3] = src[1]; break;
            case LITTLE_ENDIAN_ByteSwap: // (CDAB) -> Convert to host DCBA
                tempBuff[0] = src[1]; tempBuff[1] = src[0]; tempBuff[2] = src[3]; tempBuff[3] = src[2]; break;
        }
        memcpy(dest, tempBuff, 4);
    }
}

/**
 * @brief Computes or checks standard Modbus RTU CRC-16 checksum.
 * 
 * Uses standard Modbus polynomial 0xA001 with initial value 0xFFFF:
 * - If `Crc_Check == 1`: Compares calculated CRC with received CRC at Buffer[Length] and Buffer[Length+1].
 *   Returns 1 if valid, 0 if corrupt.
 * - If `Crc_Check == 0`: Appends calculated CRC_Low to Buffer[Length] and CRC_High to Buffer[Length+1].
 *   Returns 1.
 * 
 * @param Buffer Pointer to byte buffer.
 * @param Length Number of data bytes to calculate over.
 * @param Crc_Check 1 for checking, 0 for calculating and appending.
 * @return uint16_t 1 if valid/success, 0 if checksum failed.
 */
uint16_t ModbusCRC16(uint8_t *Buffer, uint16_t Length, uint8_t Crc_Check)
{
      uint16_t CheckSum, j, i;
      uint8_t CRC_High, CRC_Low;
      CheckSum = 0xFFFF; // Initial value

      for(j = 0; j < Length; j++)
      {
          CheckSum = CheckSum ^ (unsigned int)Buffer[j];
          for(i = 8; i > 0; i--) {
              if((CheckSum) & 0x0001) {
                  CheckSum = (CheckSum >> 1) ^ 0xa001; // XOR with Modbus polynomial
              } else {
                  CheckSum >>= 1;
              }
          }
      }
      CRC_High = (uint8_t)(CheckSum >> 8);
      CheckSum <<= 8;
      CRC_Low = (uint8_t)(CheckSum >> 8);

      if(Crc_Check == 1) {
          // Compare against received CRC bytes
          if((Buffer[Length + 1] == CRC_High) && (Buffer[Length] == CRC_Low)) return 1;
          else return 0;
      }
      else {
          // Append CRC to outgoing buffer
          Buffer[Length] = CRC_Low;
          Buffer[Length + 1] = CRC_High;
          return 1;
      }
}

/**
 * @brief Non-blocking interrupt-driven USART1 transmission over RS-485.
 * 
 * Sequence:
 * 1. Waits up to 2 seconds for previous transmission to finish.
 * 2. Clears RX buffers and counters.
 * 3. Switches transceiver to TX mode (sets PA8-DEN HIGH).
 * 4. Enables USART_IT_TXE interrupt to begin sending bytes in ISR.
 * 
 * @param send_str Pointer to data bytes to send.
 * @param send_len Length of data to send.
 */
void Modbus_Uart_Transmit(uint8_t *send_str, uint16_t send_len)
{
    // Wait for any previous transmission to finish
    int32_t wait_timeout = 2000;
    while (!g_uart_tx_complete && wait_timeout > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_timeout--;
    }
    
    g_uart_tx_complete = false;
    modbus.Rd_Cnt = 0;
    modbus.F_Rx_CHECK = 0;
    memset(modbus.Rx_Buff, 0, sizeof(modbus.Rx_Buff));
    
    tx_ptr = send_str;
    tx_len = send_len;
    
    RS485_TX_EN(); // Set transceiver to TX mode (PA8 HIGH)
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // Start TX interrupt
}

/**
 * @brief USART1 Global Interrupt Service Routine for RS-485 Modbus RTU communication.
 * 
 * Handles 3 interrupt conditions:
 * 1. RXNE (Receive Buffer Not Empty):
 *    - Reads incoming character from USART1.
 *    - Appends to `Rx_Buff`.
 *    - Automatically calculates expected response length:
 *      - Normal read response: `Rx_Buff[2] + 5` bytes (Slave + FC + ByteCount + N*Bytes + CRC_L + CRC_H).
 *      - Exception response (bit 7 of FC set): 5 bytes (Slave + FC|0x80 + ErrorCode + CRC_L + CRC_H).
 *    - Flags `F_Rx_CHECK = 1` once all expected bytes arrive.
 * 2. TXE (Transmit Data Register Empty):
 *    - Sends next byte from `tx_ptr`.
 *    - When `tx_len == 0`, disables TXE and enables TC (Transmission Complete).
 * 3. TC (Transmission Complete):
 *    - Clears TC pending bit and disables TC interrupt.
 *    - Immediately switches RS-485 transceiver back to RX mode (sets PA8-DEN LOW).
 *    - Marks `g_uart_tx_complete = true`.
 */
void USART1_IRQHandler(void) __attribute__((interrupt()));
void USART1_IRQHandler(void)
{
    // --- 1. RX Mode ---
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART1);
        
        if(modbus.Rd_Cnt < sizeof(modbus.Rx_Buff))
        {
            modbus.Rx_Buff[modbus.Rd_Cnt++] = data;
        }
        
        // Modbus frame completion detector
        if(modbus.Rd_Cnt >= 3)
        {
            uint8_t expected_len;
            if(modbus.Rx_Buff[1] & 0x80)
            {
                expected_len = 5; // Modbus Exception response (Slave + FC|0x80 + Code + CRC_L + CRC_H)
            }
            else
            {
                expected_len = modbus.Rx_Buff[2] + 5; // Normal read response (Header[3] + Data[N] + CRC[2])
            }

            if(modbus.Rd_Cnt >= expected_len)
            {
                if(!modbus.F_Rx_CHECK)
                {
                    modbus.F_Rx_CHECK = 1; // Signal packet ready for validation in iot_task
                }
            }
        }
    }
    else if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART1); // Clear overrun error flag
    }
    
    // --- 2. TX Mode (Send next byte) ---
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if(tx_len > 0)
        {
            USART_SendData(USART1, *tx_ptr++);
            tx_len--;
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // No more bytes; disable TXE
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);   // Wait for final shift-register completion
        }
    }
    
    // --- 3. TX Complete (Shift register empty) ---
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        
        RS485_RX_EN(); // Switch transceiver back to Listen/RX mode (PA8 LOW)
        g_uart_tx_complete = true;
    }
}

/**
 * @brief Initializes USART1 and GPIO pins for RS-485 Modbus RTU communication.
 * - Resolves configured baud rate (4800, 9600, 19200, 38400, 115200).
 * - Resolves configured parity (None, Even, Odd) and sets 8-bit or 9-bit word length accordingly.
 * - Resolves stop bits (1 or 2).
 * - Configures PA9 as Alternate Function Push-Pull (USART1_TX).
 * - Configures PA10 as Input Pull-Up (USART1_RX).
 * - Configures PA8 as Push-Pull Output (RS-485 Direction Enable DEN).
 * - Enables USART1 RXNE interrupt with priority (Preemption: 1, SubPriority: 0).
 */
void RS485_Init(void)
{
    uint32_t baudrate = 9600;
    uint16_t parity = USART_Parity_No;
    uint16_t stopbits = USART_StopBits_1;

    /* Map baud rate enum to integer */
    if(first.Baud_Rate == _4800){baudrate = 4800;}
    else if(first.Baud_Rate == _9600){baudrate = 9600;}
    else if(first.Baud_Rate == _19200){baudrate = 19200;}
    else if(first.Baud_Rate == _38400){baudrate = 38400;}
    else if(first.Baud_Rate == _115200){baudrate = 115200;}

    /* Map parity enum */
    if(first.Parity == _none){parity = USART_Parity_No;}
    else if(first.Parity == _even){parity = USART_Parity_Even;}
    else if(first.Parity == _odd){parity = USART_Parity_Odd;}

    /* Map stop bit */
    if(first.Stop_bit == 1){stopbits = USART_StopBits_1;}
    else if(first.Stop_bit == 2){stopbits = USART_StopBits_2;}

    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    /* Enable peripheral clocks for GPIOA and USART1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9 (TX) - Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 (RX) - Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA8 (DEN) - Transceiver Direction Enable (Push-Pull Output) */
    GPIO_InitStructure.GPIO_Pin = RS485_DEN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Default transceiver to RX (listening) mode */
    RS485_RX_EN();

    /* USART1 Configuration:
     * When parity is enabled, word length must be 9 bits (8 data bits + 1 parity bit).
     * When parity is none, word length is standard 8 bits.
     */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = (parity == USART_Parity_No) ? USART_WordLength_8b : USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = stopbits;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable RX buffer not empty interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Configure NVIC for USART1 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable USART1 peripheral */
    USART_Cmd(USART1, ENABLE);
}