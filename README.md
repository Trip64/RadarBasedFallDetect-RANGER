# RANGER — Multi-Node Fall Detection System with Radar + Wearable IMU Fusion

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Wearable Core: Nordic nRF54L15 | nRF52840](https://img.shields.io/badge/Wearable%20Core-Nordic%20nRF54L15%20%7C%20nRF52840-00A9CE.svg)](#wearable-platform-nordic-nrf54l15)
[![Fusion: Raspberry Pi Pico RP2040](https://img.shields.io/badge/Fusion-RP2040-C51A4A.svg)](#core-system-nodes)
[![Edge ML: Edge Impulse TinyML](https://img.shields.io/badge/Edge%20ML-Edge%20Impulse%20TinyML-E95420.svg)](#machine-learning--tinyml-inference-pipeline)
[![Connectivity: BLE 5.4 | 24GHz FMCW](https://img.shields.io/badge/Connectivity-BLE%205.4%20%7C%2024GHz%20FMCW-blue.svg)](#system-architecture)

---

## TL;DR

- **Two ways to catch a fall**: A wearable IMU on the body detects impact + orientation change, while a separate 24GHz FMCW radar watches from across the room. Both have to agree before triggering an alarm, which kills most false positives.
- **Nordic BLE wearables**: Built around **nRF52840** and testing on **nRF54L15** (Cortex-M33). BLE means the wearable lasts weeks on a small battery instead of hours on Wi-Fi.
- **ML runs on the MCU itself**: An Edge Impulse TinyML model runs inference at 50Hz directly on the wearable — classifies falls vs. sitting down vs. walking without needing a server.
- **Three-check confirmation**: Impact spike (>2.5g) or ML confidence (>85%), then 3 seconds of stillness, then radar cross-check. Only after all three does it trigger.
- **Camera only powers on during a confirmed fall**: The ESP32-CAM sits behind an optocoupler relay and stays completely off until a real fall closes the gate. Then it snaps a photo and sends it via Telegram.

---

## Machine Learning & TinyML Inference Pipeline

The wearable node executes an on-device Edge Impulse machine learning model optimized for Arm Cortex-M DSP instructions.

```mermaid
flowchart LR
    A["6-DoF IMU (50Hz)<br/>[ax, ay, az, gx, gy, gz]"] --> B["Signal Windowing<br/>(2000ms window / 200ms slide)"]
    B --> C["Feature Extraction<br/>Spectral Analysis (FFT)<br/>RMS Energy & Peak Jerk"]
    C --> D["Quantized TFLite Neural Network<br/>(Arm Cortex-M DSP Acceleration)"]
    D --> E["Softmax Output Classes<br/>• Fall<br/>• False Alarm (Sit/Jump)<br/>• Walking<br/>• Idle"]
    E --> F["Dual-Threshold Gating<br/>Impact > 2.5g OR ML Score > 85%"]
```

### Feature Engineering & Input Processing
1. **Kinematic Windowing**: High-rate continuous sampling of tri-axial acceleration ($a_x, a_y, a_z$) and angular rate ($g_x, g_y, g_z$) at **50 Hz** in 2.0-second overlapping time windows.
2. **Frequency & Time-Domain DSP**:
   - **Total Acceleration Vector**: $\|\vec{a}\| = \sqrt{a_x^2 + a_y^2 + a_z^2}$ to decouple body orientation from impact detection.
   - **Jerk Rate**: $\frac{d\|\vec{a}\|}{dt}$ to capture the rapid deceleration characteristic of ground impact.
   - **Spectral Power Density**: Fast Fourier Transform (FFT) frequency bin analysis to isolate chaotic tumble dynamics from periodic gait signatures.
3. **Classification Classes**:
   - `Fall`: Uncontrolled descent followed by high-g impact and rapid orientation collapse.
   - `False Alarm`: High-acceleration intentional activities (jumping, sitting down quickly, running).
   - `Walking`: Periodic rhythmic gait dynamics.
   - `Idle`: Static sitting, standing, or gentle posture shifts.

---

## Wearable Platform: Nordic nRF54L15

Wi-Fi eats too much power for something you wear all day, so the wearable side runs on **Nordic Semiconductor** BLE chips:

- **nRF52840 (current)**: Cortex-M4F @ 64MHz, 1MB Flash, 256KB RAM, BLE 5.3. This is what the wearable and base station run on right now.
- **nRF54L15 (testing — see [`NRF54_Test/`](NRF54_Test/))**:
  - Cortex-M33 @ 128MHz with better DSP instructions, so ML inference runs faster at lower power.
  - BLE 5.4 support including direction finding (AoA), which could eventually give room-level positioning.
  - Sub-microamp sleep current — realistic coin-cell battery life for months.

---

## System Architecture

```mermaid
flowchart TD
    subgraph Wearables ["Wearable Nodes"]
        WN["WearableNode<br/>(Seeed XIAO nRF52840 / nRF54L15)<br/>MPU6050/9250 + Edge Impulse ML"]
        WNS3["WatchNode S3<br/>(ESP32-S3 SuperMini)<br/>MPU9250 + Barometer"]
    end

    subgraph Stationary ["Stationary 24GHz Radar Nodes"]
        BN_RADAR["BathroomNode<br/>(ESP32-S3 + LD1125H)<br/>24GHz Zone Radar"]
    end

    subgraph Base_Processing ["Sensor Fusion & Bridge"]
        BASE["BaseNode<br/>(Seeed XIAO nRF52840)<br/>BLE Central to UART Bridge"]
        FUSION["FusionNode<br/>(Raspberry Pi Pico RP2040)<br/>RD-03D Radar + Stillness FSM"]
    end

    subgraph Display_Emergency ["Dashboard & Emergency Camera"]
        CROW["CrowPanel Advance 4.3-inch<br/>(ESP32-S3 HMI Touchscreen)<br/>800x480 LovyanGFX Display"]
        CAM["ESPCAM_Telegram<br/>(AI-Thinker ESP32-CAM)<br/>Optoisolated Relay Gated"]
    end

    WN -- "BLE Notifications (50Hz IMU / 10Hz Env / ML)" --> BASE
    BASE -- "UART 115200 Baud (CSV Telemetry)" --> FUSION
    FUSION -- "I2C Slave 0x42 (24-byte struct)" --> CROW
    WNS3 -. "ESP-NOW Broadcast (10Hz)" .-> CROW
    BN_RADAR -. "ESP-NOW Broadcast" .-> CROW
    FUSION -- "GP16 Relay Gate (Active HIGH)" --> CAM
    CROW -. "Wi-Fi HTTPS (Telegram Dispatch)" .-> CAM
```

---

## Core System Nodes

### Production Nodes ([`nodes/`](nodes/) & [`NRF54_Test/`](NRF54_Test/))

| Node | Target Hardware | Primary Sensors / Roles | Transports & Interfaces |
|---|---|---|---|
| [**WearableNode**](nodes/WearableNode/) | Seeed XIAO nRF52840 | MPU6050 6-DoF IMU, BMP280, Edge Impulse ML Engine | BLE Peripheral (Notify @ 50Hz) |
| [**NRF54_Test**](NRF54_Test/) | Seeed XIAO nRF54L15 | Cortex-M33 bring-up and I2C scanner for testing new hardware | USB-CDC, Fast-Mode I2C (400kHz), BLE 5.4 |
| [**BaseNode**](nodes/BaseNode/) | Seeed XIAO nRF52840 | Dedicated BLE Central receiver & UART bridge | BLE Central &rarr; UART (115200 baud) |
| [**FusionNode**](nodes/FusionNode/) | Raspberry Pi Pico (RP2040) | Fall FSM brain, RD-03D 24GHz mmWave radar, relay power gate | UART (256k & 115k), I2C Slave (`0x42`) |
| [**CrowPanel**](nodes/CrowPanel/) | CrowPanel Advance 4.3" (ESP32-S3) | ST7265 800x480 IPS display, GT911 touch, TCA9534 expander | I2C Master, ESP-NOW, Wi-Fi Station |
| [**WatchNode_S3**](nodes/WatchNode_S3/) | ESP32-S3 SuperMini | MPU9250 9-DoF IMU, BMP280 Barometer, WS2812B NeoPixel | Connectionless ESP-NOW (10Hz) |
| [**BathroomNode**](nodes/BathroomNode/) | ESP32-S3 SuperMini | HLK-LD1125H 24GHz radar zone presence and fall tracking | ESP-NOW Broadcast |
| [**ESPCAM_Telegram**](nodes/ESPCAM_Telegram/) | AI-Thinker ESP32-CAM | OV2640 / GC2145 image sensor, optoisolated power switch | Wi-Fi HTTPS to Telegram Bot API |

### Legacy Generation 2 System ([`legacy_ranger2/`](legacy_ranger2/))

- [**Ranger_Base_Station**](legacy_ranger2/Ranger_Base_Station/): ESP32-S3 Base Station with 320x240 ILI9341 SPI TFT, SIM800C GSM modem, and RD-03D radar.
- [**SensorNode_WiFi**](legacy_ranger2/SensorNode_WiFi/): ESP32-C6 Wi-Fi TCP wearable node streaming newline-delimited JSON.
- [**ShowNode_Auxiliary**](legacy_ranger2/ShowNode_Auxiliary/): Deneyap 1A v2 (ESP32-S3) demonstration node with SSD1306 OLED, DHT11, and radar.

### Utilities & Tools ([`tools/`](tools/))

- [**EI_DataCollector**](tools/EI_DataCollector/): Streamlined Arduino IMU CSV logger with automated `capture.sh` script for training Edge Impulse models.
- [**RadarOverlay**](tools/RadarOverlay/): Canvas-based real-time 2D radar point-cloud visualizer.

---

## Fall Verification Logic

```mermaid
flowchart TD
    A["IMU Peak Impact > 2.5g<br/>OR Edge Impulse ML Score > 85%"] --> B["3-Second Stillness Window<br/>(|a| - 1.0g < 0.25g for 25 samples)"]
    B -->|Stillness Confirmed| C["CONFIRMED FALL"]
    B -->|Movement Detected| D["FALSE ALARM / RESET"]
    C --> E["Tactical HMI Alarm"]
    C --> F["Relay Closes ESP-CAM Gate"]
    C --> G["Telegram Bot Dispatch"]
    D --> H["Return to Monitor State"]
```

---

## Project Roadmap & TODO

- [ ] **Port ML model to nRF54L15**: Get Edge Impulse inference and CMSIS-NN kernels running on the Cortex-M33.
- [ ] **BLE direction finding**: Use AoA to figure out which room the wearable is in.
- [ ] **Barometric fall detection**: Use BMP388 altitude drops ($\Delta h > 0.8\text{m}$ in $< 500\text{ms}$) as another ML feature.
- [ ] **Wake-on-motion sleep**: Use IMU `WOM` interrupt so the Nordic chip stays in deep sleep until actual movement happens.
- [ ] **More training data**: Record more real-world stumbles, slips, and edge cases for Edge Impulse.
- [ ] **Encrypted BLE**: Add AES-128 pairing between wearable and base station.

---

## Directory Structure

- [`docs/`](docs/) — Technical specifications and theoretical foundations:
  - [`ARCHITECTURE.md`](docs/ARCHITECTURE.md) — System topology, bus specifications, and packet byte maps
  - [`PHYSICS_THEORY.md`](docs/PHYSICS_THEORY.md) — Kinematics, FMCW radar Doppler equations, and RF link budget
  - [`PINOUT_GUIDE.md`](docs/PINOUT_GUIDE.md) — Full pinout and wiring tables across all boards
- [`nodes/`](nodes/) — Source code for active production nodes
- [`NRF54_Test/`](NRF54_Test/) — Hardware bring-up and I2C probing for the nRF54L15
- [`legacy_ranger2/`](legacy_ranger2/) — Generation 2 reference code and prototype modules
- [`tools/`](tools/) — Dataset acquisition scripts and radar visualization tools

---

## Getting Started & Flashing Instructions

### Required Arduino Board Packages

- **Nordic nRF52:** `Seeed nRF52 Boards` (v1.1.8+) or `Adafruit nRF52`
- **Nordic nRF54:** `Seeed nRF54 Boards` / Zephyr RTOS toolchain
- **Raspberry Pi Pico:** `Raspberry Pi Pico/RP2040` by Earle F. Philhower, III
- **ESP32 Core:** `esp32` by Espressif Systems (v2.0.14+ or v3.0+)

### Required Libraries

- `LovyanGFX` (for CrowPanel ST7265 RGB parallel display)
- `TCA9534` (for CrowPanel port expander)
- `DFRobotDFPlayerMini` (for audio synthesizer on Fusion Node)
- `Adafruit NeoPixel` (for RGB LED indicators)
- `Adafruit GFX` & `Adafruit ILI9341` / `Adafruit SSD1306` (for legacy nodes)

---

## Configuration & Credentials

Before flashing network-connected nodes ([`CrowPanel.ino`](nodes/CrowPanel/CrowPanel.ino), [`ESPCAM_Telegram.ino`](nodes/ESPCAM_Telegram/ESPCAM_Telegram.ino), and [`Ranger.ino`](legacy_ranger2/Ranger_Base_Station/Ranger.ino)), configure your network credentials in each respective file:

```cpp
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"
#define TELEGRAM_BOT_TOKEN  "YOUR_TELEGRAM_BOT_TOKEN"
#define TELEGRAM_CHAT_ID    "YOUR_TELEGRAM_CHAT_ID"
```

---

## Technical Documentation

- [System Architecture & Data Format Specification](docs/ARCHITECTURE.md)
- [Physics & RF Link Budget Theory](docs/PHYSICS_THEORY.md)
- [Hardware Wiring & Pinout Guide](docs/PINOUT_GUIDE.md)

---

## License

This project is licensed under the MIT License — see the repository root for details.
