# Bootloader_UART_STM32
Load firmware in to MCU from another MCU through UART.
I have taken the .bin file from project and converted the file in to array file and i have transferted to MCU through the UART.

This project demonstrates how to load firmware into an STM32 microcontroller from another STM32 MCU using UART, utilizing the built-in STM32 bootloader protocol.

## 🔧 Overview

This setup allows one STM32 microcontroller (acting as a **master programmer**) to send a firmware `.bin` file to another STM32 microcontroller (the **target**) via UART, using the STM32 bootloader interface.

The target MCU must be in **bootloader mode** (BOOT0 pin set appropriately) to accept firmware updates over UART.

## 📦 Key Features

- Transfers firmware over UART (USART) using STM32’s system bootloader
- `.bin` firmware file is converted into a C array and stored in the master MCU's flash/ROM
- Supports erase, write, and verification over UART
- No external programmer needed

## 🔁 Workflow

1. Build the target application and extract the `.bin` file.
2. Convert the `.bin` file into a C array using a binary-to-header converter (e.g. `xxd -i` or custom script).
3. Include the generated array in the master MCU project.
4. Use the master STM32 to:
   - Reset the target into bootloader mode
   - Communicate via UART using the STM32 bootloader protocol
   - Erase target flash
   - Write the firmware data
   - Optionally verify and reset

## 🧪 Requirements

- 2 STM32 development boards (Master and Target)
- UART interface connected between the two (Tx ↔ Rx, GND shared)
- BOOT0 pin access on the target STM32
- Pull-up resistors as needed
- STM32CubeIDE / STM32CubeMX for firmware development

## 🔌 UART Bootloader Protocol Used

The master MCU sends commands like:

- `0x7F` — Start command
- `0x11` — Read memory
- `0x31` — Write memory
- `0x43` — Erase memory

These follow the STM32 bootloader protocol (see ST AN3155).

## 🚀 How to Use

1. Prepare the `.bin` file from the target firmware build.
2. Convert it into C array:
   ```bash
   xxd -i firmware.bin > firmware_data.h

