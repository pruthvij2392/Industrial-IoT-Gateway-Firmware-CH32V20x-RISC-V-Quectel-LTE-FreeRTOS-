#ifndef __RS485_H
#define __RS485_H

#include "ch32v20x.h"
#include <stdio.h>
#include <stdbool.h>

/* ==============================================================================
 * RS-485 TRANSCEIVER DIRECTION PIN (PA8)
 * ==============================================================================
 * Half-duplex RS-485 transceivers (such as MAX485, SP3485, or SN65HVD) require
 * Driver Enable (DE) and Receiver Enable (/RE) lines.
 * Both pins are tied to MCU pin PA8:
 * - HIGH: Enables Transmitter driver (TX mode).
 * - LOW:  Enables Receiver driver (RX mode).
 */
#define RS485_DEN_PORT   GPIOA
#define RS485_DEN_PIN    GPIO_Pin_8
#define RS485_TX_EN()    GPIO_SetBits(RS485_DEN_PORT, RS485_DEN_PIN)   /**< Set PA8 HIGH for TX */
#define RS485_RX_EN()    GPIO_ResetBits(RS485_DEN_PORT, RS485_DEN_PIN) /**< Set PA8 LOW for RX */

/* CRC operation mode flags */
#define CRC_CHECK   1  /**< Verify CRC of incoming response frame against calculated CRC */
#define CRC_GET     0  /**< Calculate CRC and append bytes to outgoing request frame */

#define number_of_slave 16 /**< Maximum number of Modbus slave devices / polling slots */

/**
 * @brief Modbus Byte-Order / Endianness Models:
 * Used to interpret multi-byte numbers (16-bit and 32-bit integers, 32-bit IEEE 754 floats).
 */
typedef enum {
    BIG_Endian,             /**< ABCD: Standard Big Endian (Most Significant Word first, MSB first) */
    LITTLE_Endian,          /**< DCBA: Standard Little Endian (Least Significant Word first, LSB first) */
    BIG_ENDIAN_ByteSwap,    /**< BADC: Big Endian with Byte Swap (Mid-Little Endian) */
    LITTLE_ENDIAN_ByteSwap  /**< CDAB: Little Endian with Byte Swap (Mid-Big Endian) */
} ModbusEndian_t;

/**
 * @brief Modbus Data Types supported for register decoding:
 */
typedef enum {
    TYPE_UINT16, /**< 16-bit Unsigned Integer (1 register) */
    TYPE_INT16,  /**< 16-bit Signed Integer (1 register) */
    TYPE_UINT32, /**< 32-bit Unsigned Integer (2 registers) */
    TYPE_INT32,  /**< 32-bit Signed Integer (2 registers) */
    TYPE_FLOAT   /**< 32-bit IEEE 754 Single-Precision Floating Point (2 registers) */
} MBDataType_t;

/**
 * @brief Supported RS-485 Baud Rates:
 */
typedef enum {
    _4800,   /**< 4800 bps */
    _9600,   /**< 9600 bps (standard industrial default) */
    _19200,  /**< 19200 bps */
    _38400,  /**< 38400 bps */
    _115200  /**< 115200 bps */
} Modbusbaudrate_t;

/**
 * @brief Supported RS-485 Parity options:
 */
typedef enum {
    _zero, /**< Forced 0 / Space parity */
    _none, /**< No parity bit (8 data bits, 1 or 2 stop bits) */
    _even, /**< Even parity */
    _odd,  /**< Odd parity */
} Modbusparity_t;

/**
 * @brief Runtime state and buffers for the RS-485 Modbus RTU Master engine.
 */
typedef struct 
{
    bool F_Rx_CHECK;        /**< Set to 1 by USART1 RX interrupt when expected response bytes are received */
    uint8_t Rx_Buff[70];    /**< Buffer storing incoming Modbus RTU response packet */
    uint8_t Tx_Buff[10];    /**< Buffer storing outgoing Modbus RTU request frame (8 bytes standard) */
    uint8_t Rd_Cnt;         /**< Number of bytes currently received in Rx_Buff */
    uint8_t Send_TM;        /**< Transmission timing counter */
    uint8_t Device_Cnt;     /**< Round-robin index of active slave being polled (0 to No_Of_Slave - 1) */
    uint8_t Dno;            /**< Active device slot number during ongoing transaction */
    uint8_t Mode;           /**< Polling state: 1 = Send request, 2 = Waiting for response */
    uint8_t RowNo;          /**< Variable/row index being decoded */
    uint8_t RegNo;          /**< Register offset index */
    uint8_t RdFC;           /**< Function Code being processed */
    uint8_t RdDeviceNo;     /**< Device slot number during JSON configuration */
    uint8_t RdQnty;         /**< Quantity of registers requested */
    uint16_t Tm_count;      /**< Timeout counter for response detection */
    uint16_t RdSlaveID_1;   /**< Temporary slave ID holder */
    uint16_t RdSlaveID_3;   /**< Temporary slave ID holder */
    uint16_t RdSlaveAdd_1;  /**< Temporary start address holder */
    uint16_t RdSlaveAdd_2;  /**< Temporary start address holder */
    uint16_t RdSlaveVal;    /**< Temporary register value holder */
    uint8_t HoldReg[16][64];/**< Raw register byte storage table: 16 slaves x 64 bytes */
    uint8_t *HoldAdd[64];   /**< Pointer lookup array pointing to specific register byte addresses */
    uint8_t DataType;       /**< Active decoded data type */
    uint8_t Decimal;        /**< Active decimal point position */
    uint8_t Endian;         /**< Active endianness model */
    uint8_t buffCnt;        /**< Index of variable being formatted */
    uint8_t Error_Cnt[16];  /**< Consecutive communication error/timeout counter per slave */
    uint8_t RdSlaveID_2;    /**< Slave ID parsed from incoming JSON config frame */
    uint16_t U16_Value[16]; /**< Decoded UINT16 values for all 16 channels */
    int16_t I16_Value[16];  /**< Decoded INT16 values for all 16 channels */
    uint32_t U32_Value[16]; /**< Decoded UINT32 values for all 16 channels */
    int32_t I32_Value[16];  /**< Decoded INT32 values for all 16 channels */
    float F32_Value[16];    /**< Decoded 32-bit floating point values for all 16 channels */

} modbusRTU;

extern modbusRTU modbus;

/**
 * @brief Constructs and transmits a Modbus RTU Read Request for slave `DeviceNo`.
 * @param DeviceNo Device slot index (0 to 15).
 */
void Modbus_Send(uint8_t DeviceNo);

/**
 * @brief Validates CRC-16 of received Modbus RTU packet and copies registers to holding table.
 */
void Modbus_Data_Rx(void);

/**
 * @brief Initializes USART1 (PA9-TX, PA10-RX), direction pin (PA8-DEN), and NVIC for Modbus RTU.
 */
void RS485_Init(void);

/**
 * @brief Parses raw Modbus register bytes into values and formats ASCII string for telemetry.
 */
void Modbus_LCDPrint(void);

#endif /* __RS485_H */

