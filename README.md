# STM32 FM Receiver with QN8035

This project implements an FM receiver using the STM32 microcontroller and QN8035 FM tuner chip.

## Hardware Features

- MCU: STM32F103 (Blue Pill)
- FM Tuner: QN8035
- Display: OLED SSD1306 128x64 (I2C)
- Input: Rotary encoder with button, UP/DOWN buttons, POWER button
- Interface: I2C for tuner and display control

## Specifications

- FM Band: 60-110 MHz
- Volume Control: 8 levels (0-7)
- RDS Support: Yes
- Power Supply: Battery powered with voltage monitoring
- Display Info: Frequency, Volume, Stereo/Mono, Signal Quality (S/N), RSSI, RDS text

## Controls

- Rotary Encoder:
  - Rotate: Change frequency (0.05 MHz steps) or volume
  - Push: Switch between frequency/volume control modes
- UP Button: Toggle Stereo/Mono mode
- DOWN Button: Decrease frequency
- POWER Button: Power off device
- UP + DOWN (held): Power off device

## Building and Flashing

This project uses PlatformIO for building and flashing. To build:

1. Install PlatformIO
2. Open the project in PlatformIO
3. Build and upload to your STM32F103 board

## Pin Configuration

- PB0: POWER Button (INPUT_PULLUP)
- PB1: DOWN Button (INPUT_PULLUP)
- PB2: UP Button (INPUT_PULLUP)
- PA15: LED indicator
- PB10: LED indicator
- PA5: Encoder Button
- PA6/PB7: Encoder inputs
- I2C Pins (for OLED and QN8035):
  - PB6: SCL
  - PB7: SDA

## Features

- Digital volume control
- RDS text display
- Signal quality indication
- Battery level monitoring
- Stereo/Mono switching
- Fine-tuning with encoder
- Non-volatile settings storage
- Power saving mode
