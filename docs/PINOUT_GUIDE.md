# RANGER Hardware Pinout & Wiring Guide

This document specifies the complete pin assignments, hardware buses, communication interfaces, and power distribution across all nodes in the RANGER ecosystem.

---

## Quick Navigation

- [1. WearableNode (Seeed XIAO nRF52840)](#1-wearablenode-seeed-xiao-nrf52840)
- [2. BaseNode (Seeed XIAO nRF52840)](#2-basenode-seeed-xiao-nrf52840)
- [3. FusionNode (Raspberry Pi Pico RP2040)](#3-fusionnode-raspberry-pi-pico-rp2040)
- [4. CrowPanel Advance 4.3" (ESP32-S3)](#4-crowpanel-advance-43-esp32-s3)
- [5. WatchNode S3 (ESP32-S3 SuperMini)](#5-watchnode-s3-esp32-s3-supermini)
- [6. BathroomNode (ESP32-S3 SuperMini)](#6-bathroomnode-esp32-s3-supermini)
- [7. ESPCAM_Telegram (AI-Thinker ESP32-CAM)](#7-espcam_telegram-ai-thinker-esp32-cam)

---

## 1. WearableNode (Seeed XIAO nRF52840)

Primary wearable node for high-frequency kinematic capture and on-device Edge Impulse ML fall inference.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| I2C SDA | `D4` (`P0.04`) | Bidirectional | I2C Data (400 kHz Fast Mode) | MPU6050 (0x68), BMP280 (0x76) |
| I2C SCL | `D5` (`P0.05`) | Output | I2C Clock (400 kHz Fast Mode) | MPU6050 (0x68), BMP280 (0x76) |
| Emergency SOS Button | `D1` (`P0.03`) | Input | Digital Input (`INPUT_PULLUP`) | Active LOW to GND (3s hold trigger) |
| Status LED | `PIN_LED` (`P0.26`) | Output | Digital Output | Active LOW onboard blue LED |
| Battery Voltage Sense | `A0` (`P0.02`) | Input | Analog In (Internal Divider) | 12-bit ADC for LiPo monitoring |
| Power In | `3V3` / `GND` | Power | 3.3V DC Regulated | LiPo Battery / USB-C supply |

---

## 2. BaseNode (Seeed XIAO nRF52840)

Dedicated BLE Central node acting as a low-latency wireless-to-UART bridge for the wearable.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| UART TX to Fusion Node | `D6` (`P0.06`) | Output | Hardware Serial1 TX (115200 baud, 8N1) | Connects to Fusion Node GP1 (RX) |
| UART RX from Fusion Node | `D7` (`P0.07`) | Input | Hardware Serial1 RX (115200 baud, 8N1) | Connects to Fusion Node GP0 (TX) |
| Status LED | `PIN_LED` (`P0.26`) | Output | Digital Output | Active LOW onboard LED (BLE link status) |
| Power In | `5V` / `GND` | Power | 5V DC Bus | Shared system power bus |

---

## 3. FusionNode (Raspberry Pi Pico RP2040)

Central real-time processor executing the multi-target FMCW radar tracking, fall verification state machine, audio alarms, and optoisolated relay gate.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| Base UART TX | `GP0` | Output | `Serial1` TX (115200 baud) | Connects to Base Node D7 (RX) |
| Base UART RX | `GP1` | Input | `Serial1` RX (115200 baud) | Connects to Base Node D6 (TX) |
| I2C1 SDA (CrowPanel Slave) | `GP2` | Bidirectional | `Wire1` Data (Slave Address `0x42`) | Connects to CrowPanel GPIO 15 |
| I2C1 SCL (CrowPanel Slave) | `GP3` | Input | `Wire1` Clock (Slave Address `0x42`) | Connects to CrowPanel GPIO 16 |
| Radar UART TX | `GP4` | Output | `Serial2` TX (256000 baud) | Connects to RD-03D RX |
| Radar UART RX | `GP5` | Input | `Serial2` RX (256000 baud) | Connects to RD-03D TX |
| Audio Synth TX | `GP8` | Output | `SerialPIO` Software Serial (9600 baud) | Connects to DFPlayer Mini RX (via 1k resistor) |
| Audio Synth RX | `GP9` | Input | `SerialPIO` Software Serial (9600 baud) | Connects to DFPlayer Mini TX |
| Status LED Red | `GP13` | Output | Digital Output | Active HIGH RGB status indicator |
| Status LED Green | `GP14` | Output | Digital Output | Active HIGH RGB status indicator |
| Status LED Blue | `GP15` | Output | Digital Output | Active HIGH RGB status indicator |
| ESP32-CAM Relay Gate | `GP16` | Output | Digital Output | Active HIGH (closes optoisolated power relay) |
| Piezo Buzzer | `GP22` | Output | PWM / Tone Output | High-output audible local alarm |
| Power In | `VSYS` / `GND` | Power | 5.0V DC Input | Main 5V power rail |

---

## 4. CrowPanel Advance 4.3" (ESP32-S3)

800x480 high-resolution IPS touch dashboard displaying real-time kinematics, radar point clouds, and Wi-Fi Telegram bot notifications.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| I2C SDA (Master) | `GPIO 15` | Bidirectional | I2C Fast Mode (400 kHz) | Shared: GT911 Touch + TCA9534 + Fusion Node |
| I2C SCL (Master) | `GPIO 16` | Output | I2C Fast Mode (400 kHz) | Shared: GT911 Touch + TCA9534 + Fusion Node |
| User Input Button | `GPIO 6` | Input | Digital Input (`INPUT_PULLUP`) | Onboard user button (Active LOW) |
| RGB Parallel Data D0..D15 | `GPIO 21, 47, 48, 45, 38, 9..14, 7, 17, 18, 3, 46` | Output | 16-bit RGB565 Parallel Bus | ST7265 IPS LCD Controller interface |
| LCD Synchronization | `GPIO 42` (DE), `41` (VSYNC), `40` (HSYNC), `39` (PCLK) | Output | Display Timing Signals | High-speed pixel clock & frame sync |
| I2C Port Expander TCA9534 | `0x18` (I2C) | Control | Register-mapped I/O | Port 1: Backlight Enable, Port 2: GT911 Reset |
| Power Supply | `5V` / `GND` | Power | 5V DC (min 2A recommended) | USB-C or terminal block |

---

## 5. WatchNode S3 (ESP32-S3 SuperMini)

Alternative wrist-worn sensor node utilizing ESP-NOW connectionless broadcast with a 9-DoF IMU and barometric altimeter.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| I2C SDA | `GPIO 8` | Bidirectional | I2C Data (400 kHz) | MPU9250 (`0x68`), AK8963 (`0x0C`), BMP280 (`0x76`) |
| I2C SCL | `GPIO 9` | Output | I2C Clock (400 kHz) | MPU9250 (`0x68`), AK8963 (`0x0C`), BMP280 (`0x76`) |
| BOOT / SOS Button | `GPIO 0` | Input | Digital Input (`INPUT_PULLUP`) | Active LOW manual emergency trigger |
| Status LED | `GPIO 2` | Output | Digital Output | Active HIGH onboard blue LED |
| Addressable NeoPixel | `GPIO 48` | Output | WS2812B Protocol | Multi-color operational status LED |
| Power In | `5V` / `3V3` / `GND` | Power | 3.7V LiPo / 5V USB | Portable battery powered |

---

## 6. BathroomNode (ESP32-S3 SuperMini)

Dedicated wall-mounted stationary radar node monitoring high-risk private zones (e.g. bathroom, shower) where cameras are prohibited.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| Radar UART RX | `GPIO 6` | Input | Hardware Serial RX (115200 baud) | Connects to HLK-LD1125H TX |
| Radar UART TX | `GPIO 5` | Output | Hardware Serial TX (115200 baud) | Connects to HLK-LD1125H RX |
| Piezo Buzzer | `GPIO 10` | Output | Tone / PWM Output | Local warning buzzer |
| Addressable NeoPixel | `GPIO 48` | Output | WS2812B Protocol | Zone presence / alarm status indicator |
| Power In | `5V` / `GND` | Power | 5V DC Input | USB / 5V DC Wall adapter |

---

## 7. ESPCAM_Telegram (AI-Thinker ESP32-CAM)

Privacy-first emergency camera node. Remains completely unpowered until energized by the Fusion Node relay upon a confirmed fall event.

| Function | Pin / GPIO | Direction | Protocol / Electrical Spec | Notes |
|---|---|---|---|---|
| Camera Data Bus Y2..Y9 | `GPIO 5, 18, 19, 21, 36, 39, 34, 35` | Input | Parallel DVP Image Bus | OV2640 / GC2145 sensor data |
| Camera Clocks & Sync | `GPIO 0` (XCLK), `22` (PCLK), `25` (VSYNC), `23` (HREF) | Bidirectional | DVP Timing & Clock Lines | Image sensor capture synchronization |
| Camera SCCB (I2C) | `GPIO 26` (SDA), `GPIO 27` (SCL) | Bidirectional | SCCB Control Bus | Sensor register configuration |
| Sensor Power Down | `GPIO 32` | Output | Digital Control | Power gating control for image sensor |
| Main VCC Power | `5V` / `GND` | Power | Switched 5V Supply | Controlled via Fusion Node `GP16` Optocoupler Relay |
