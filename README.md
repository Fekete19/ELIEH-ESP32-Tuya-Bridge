# ELIEH ESP32 Tuya Bridge

ELIEH ESP32 Tuya Bridge is a lightweight UART bridge library for ESP32 devices.

It automatically detects and assigns:

- Tuya MCU modules (binary UART protocol)
- A1S / voice / text modules (tag-based UART protocol)

The library is designed for smart home controllers, voice assistants, and hybrid AI + Tuya systems.

---

## Features

- Automatic UART device detection
- Supports 2 hardware UART ports
- A1S framed communication (`<STT>...</STT>`, `<SAY>...</SAY>`, etc.)
- Tuya binary packet detection (`0x55 0xAA`)
- Non-blocking UART handling
- Simple API
- Modular profiles (MIN / MID / MAX)
- Works on any ESP32 (ESP32 / S2 / S3 / C3)

---

## Typical use case

ESP32 connected to:

- Tuya MCU (SmartLife / Tuya Cloud)
- Voice module (A1S or similar)

ESP32 automatically detects which UART is Tuya and which is A1S.

---

## Installation

Copy the `ELIEH_UniBridgeUART` folder into your Arduino `libraries` directory.

Then include:

```cpp
#include <ELIEH_UniBridgeUART.h>
