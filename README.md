# Industrial IoT Gateway Firmware (CH32V20x RISC-V + Quectel LTE + FreeRTOS)

Comprehensive embedded firmware for an **Industrial IoT Gateway** developed for the **WCH CH32V203** microcontroller (QingKe RISC-V V4F core @ 144 MHz) running **FreeRTOS V202112.00**, communicating with a **Quectel LTE Cat-1 cellular modem with GNSS**, polling industrial sensors over **RS485 Modbus RTU**, and logging telemetry data onto an external **W25Q SPI Flash** memory with **USB CDC** diagnostics.

---

## 🌟 Key Features

- **Microcontroller**: WCH CH32V203 (RISC-V 32-bit QingKe V4F Core, 64KB Flash, 20KB SRAM).
- **Real-Time OS**: FreeRTOS V202112.00 pre-emptive multitasking with 500 Hz tick rate (2 ms tick).
- **Cellular & Cloud**: Quectel LTE Cat-1 Modem with integrated GNSS positioning.
  - Non-blocking AT command FSM for Echo cancellation, SIM detection, registration, and time synchronization.
  - MQTT Client supporting automated Connect, Subscribe to Remote Configuration topic, and Telemetry Publishing.
  - **Dual SIM Automatic Failover**: Monitors CSQ signal quality; dynamically switches SIM slot (`AT+QDSIM=0/1`) when signal quality drops below acceptable thresholds.
- **Industrial Sensor Bus**: RS485 Modbus RTU Master (USART1).
  - Round-robin polling of up to 16 Modbus Slaves (Function Codes 0x03 & 0x04).
  - Supports INT16, UINT16, INT32, UINT32, and IEEE-754 Single-Precision FLOAT data types.
  - Configurable Endianness models: Big-Endian, Little-Endian, Big-Endian Byte Swap, Little-Endian Byte Swap.
- **Persistent Data Logging**: Winbond W25Q external SPI Flash (32MB).
  - Power-fail recovery with wear-leveling circular buffer storing up to **131,000 telemetry frames**.
  - Sector management and safe concurrent access protected by FreeRTOS Mutex.
- **Precision Timekeeping**: Maxim DS3231 I2C High-Accuracy Real-Time Clock.
  - 1 Hz Square Wave (SQW) output connected to external interrupt (`EXTI0` on PA0) for jitter-free 1-second system tick synchronization.
  - Network time synchronization via cellular NITZ (`AT+QLTS=2`).
- **USB CDC Virtual COM Port**:
  - Full-speed USB CDC interface for on-site field configuration, JSON command processing, and bulk CSV telemetry streaming (`$$$START_STREAM$$$`).

---

## 📁 Project Directory Structure

```
IOT_GATEWAY_V6/
├── Core/               # RISC-V QingKe V4F core support files
├── Debug/              # Debug and diagnostic drivers
├── FreeRTOS/           # FreeRTOS kernel source and RISC-V port
├── Ld/                 # Linker scripts (.ld)
├── Peripheral/         # WCH CH32V20x Standard Peripheral Drivers
├── Startup/            # RISC-V startup assembly (startup_ch32v20x_D8W.S)
├── User/               # Application source code
│   ├── main.c          # System entry point, clock, and FreeRTOS task startup
│   ├── comman.h / .c   # Global structs, default parameters, LED GPIO drivers
│   ├── iot_task.h / .c # Main telemetry orchestration task (Modbus + GSM FSM)
│   ├── quectel_ioc.h/.c# Quectel LTE AT engine, Dual SIM, GNSS, MQTT FSM
│   ├── rs485.h / .c    # RS485 USART1 Modbus RTU Master driver
│   ├── w25qxx.h / .c   # W25Q SPI Flash circular log storage engine
│   ├── ds3231.h / .c   # DS3231 I2C RTC driver & EXTI0 1Hz interrupt
│   ├── custom_json.c   # Zero-allocation in-place JSON parser
│   └── usb_cdc_task.h/.c # USB CDC task for field configuration & data dump
├── .project            # Eclipse / MounRiver Studio project definition
└── IOT_GATEWAY_V6.wvproj# MounRiver Studio workspace project
```

---

## ⚙️ Pinout Mapping

| Peripheral | Signal | Pin | Description |
| :--- | :--- | :--- | :--- |
| **Quectel Modem** | USART2 TX | PA2 | AT Command Transmit (115200 bps) |
| | USART2 RX | PA3 | AT Command Receive |
| | PWRKEY | PB3 | Modem Power Key Pulse (Remapped from JTAG) |
| | POWER EN | PB4 | VBAT Power Switch Enable (Remapped from JTAG) |
| **RS485 Bus** | USART1 TX | PA9 | Modbus Master Transmit |
| | USART1 RX | PA10 | Modbus Master Receive |
| | DIR (RE/DE) | PA8 | Direction Control (High = TX, Low = RX) |
| **SPI Flash (W25Q)** | SPI1 SCK | PA5 | SPI Clock |
| | SPI1 MISO | PA6 | Master In Slave Out |
| | SPI1 MOSI | PA7 | Master Out Slave In |
| | CS | PA4 | Chip Select (Active Low) |
| **DS3231 RTC** | I2C1 SCL | PB6 | I2C Clock |
| | I2C1 SDA | PB7 | I2C Data |
| | 1Hz SQW | PA0 | External Interrupt 0 (`EXTI0`) |
| **Diagnostic LEDs** | NW_LED | PB12 | Cellular Network Status |
| | DATA_LED | PB13 | MQTT & Telemetry Traffic |
| | GPS_LED | PB14 | GNSS Fix Status |
| **USB CDC** | USB DP / DM | PA11 / PA12 | Full Speed USB Virtual COM Port |

---

## 🛠️ Build & Toolchain

- **IDE**: [MounRiver Studio](http://www.mounriver.com/) (MRS) or Eclipse CDT with RISC-V plugin.
- **Compiler**: `riscv-none-embed-gcc` (RISC-V QingKe V4F Architecture).
- **Optimization**: `-O1` or `-O2`, `-msmall-data-limit=8`, `-mno-save-restore`.

---

## 📜 License

This firmware is distributed for industrial IoT automation and gateway applications.
