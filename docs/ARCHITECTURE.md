# RANGER Architecture & Protocol Specification

This document details the system topology, inter-node communication protocols, packet byte layouts, and the multi-stage fall verification state machine.

---

## System Topology & Flow

```mermaid
flowchart TD
    subgraph Wearables ["Ultra-Low-Power Wearables"]
        WN["WearableNode<br/>(Seeed XIAO nRF52840 / nRF54L15)<br/>MPU6050/9250 + Edge Impulse ML"]
        WNS3["WatchNode S3<br/>(ESP32-S3 SuperMini)<br/>MPU9250 + Mag + Baro"]
    end

    subgraph Stationary ["Stationary Zone Monitors"]
        BN_RADAR["BathroomNode<br/>(ESP32-S3 + LD1125H)<br/>24GHz Zone Radar"]
    end

    subgraph Base_Processing ["Sensor Fusion & Bridge"]
        BASE["BaseNode<br/>(Seeed XIAO nRF52840)<br/>BLE Central to UART Bridge"]
        FUSION["FusionNode<br/>(Raspberry Pi Pico RP2040)<br/>Multi-Target Radar & Stillness FSM"]
    end

    subgraph Display_Emergency ["Tactical Dashboard & Emergency Node"]
        CROW["CrowPanel Advance 4.3-inch<br/>(ESP32-S3 HMI Touchscreen)<br/>800x480 Tactical LovyanGFX"]
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

## Communication Protocols

### 1. BLE Characteristic Specification (WearableNode &rarr; BaseNode)

- **Service UUID:** `833d1814-9988-4e31-8db2-2c67699cd1c1`

#### IMU Characteristic (`...2a01`) — 20 Bytes @ 50 Hz

| Byte Range | Field | Data Type | Scaling / Representation |
|---|---|---|---|
| `[0..5]` | Accelerometer ($a_x, a_y, a_z$) | `int16_t` (Big-Endian) | Divide by $4096 \text{ LSB/g}$ |
| `[6..11]` | Gyroscope ($g_x, g_y, g_z$) | `int16_t` (Big-Endian) | Divide by $65.5 \text{ LSB/dps}$ |
| `[12..17]` | Magnetometer ($m_x, m_y, m_z$) | `int16_t` (Big-Endian) | Raw magnetic flux density |
| `[18..19]` | Sequence Counter | `uint16_t` (Big-Endian) | Monotonically increasing packet ID |

#### Environmental Characteristic (`...2a02`) — 15 Bytes @ 10 Hz

| Byte Range | Field | Data Type | Scaling / Description |
|---|---|---|---|
| `[0..3]` | Barometric Pressure | `int32_t` (Big-Endian) | Pressure in Pascals ($\text{Pa}$) |
| `[4..7]` | Photoplethysmography IR | `uint32_t` (Big-Endian) | MAX30102 Infrared ADC Count |
| `[8..11]` | Photoplethysmography Red | `uint32_t` (Big-Endian) | MAX30102 Red ADC Count |
| `[12]` | Status Byte | `uint8_t` | Bit 7: SOS state (`1`=Active), Bits 0..6: Battery % (`0..100`) |
| `[13..14]` | Sequence Counter | `uint16_t` (Big-Endian) | Monotonically increasing packet ID |

#### Machine Learning Classifier Characteristic (`...2a04`) — 8 Bytes

| Byte Index | Field | Range / Description |
|---|---|---|
| `[0]` | Fall Score | `0 – 100%` Confidence |
| `[1]` | False Alarm Score | `0 – 100%` Confidence |
| `[2]` | Idle Score | `0 – 100%` Confidence |
| `[3]` | Walking Score | `0 – 100%` Confidence |
| `[4]` | Winner Class Index | `0`: Fall, `1`: False Alarm, `2`: Idle, `3`: Walking |
| `[5]` | Prediction Confidence | Overall model certainty percentage |
| `[6]` | Status Flags | Bit 0 = SOS State |
| `[7]` | Sequence Counter | Monotonic sequence byte |

---

### 2. BaseNode &rarr; FusionNode UART Frame Format (115200 Baud)

The BaseNode decodes incoming BLE packets and forwards ASCII CSV telemetry strings over Hardware UART (`Serial1`):

- **Kinematics Line:** `I,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<seq>\n`
- **Environmental Line:** `E,<pressure_pa>,<ir>,<red>,<battery>,<sos>,<seq>\n`
- **Machine Learning Line:** `M,<fall%>,<false_alarm%>,<idle%>,<walk%>,<winner_idx>,<confidence%>,<flags>\n`
- **System Status Events:** `STAT,BASE_BOOT`, `STAT,BASE_LINK_OK`, `STAT,W_OK`, `STAT,W_LOST`, `STAT,LINK_LOST`

---

### 3. FusionNode &rarr; CrowPanel I2C Slave Specification (Address 0x42)

The Fusion Node acts as an I2C slave responder (`Wire1`, address `0x42`), returning a packed **24-byte telemetry structure** with a 1-byte XOR checksum on every read request from the CrowPanel master.

```cpp
struct TelemetryPacket {
    uint8_t  magic;         // 0xAA preamble
    int16_t  ax, ay, az;    // Raw acceleration (divide by 4096 -> g)
    int16_t  gx, gy, gz;    // Raw angular velocity (divide by 65.5 -> dps)
    uint8_t  battery;       // 0–100%
    uint8_t  sos;           // 0 or 1
    uint8_t  fallState;     // 0 = Normal, 1 = Verifying, 2 = Confirmed Fall
    uint8_t  wearLink;      // 0 = Offline, 1 = Connected
    uint8_t  radarPresent;  // 0 = Clear, 1 = Target Tracked
    int16_t  radarDist_mm;  // Target Distance in millimeters
    int16_t  radarSpeed;    // Target Velocity in cm/s
    uint8_t  radarTargets;  // Active Target Count (0–3)
    uint8_t  checksum;      // XOR checksum of bytes 0..22
};
```

---

### 4. RD-03D 24GHz mmWave Radar Interface (256000 Baud)

- **Frame Header:** `0xAA 0xFF 0x03 0x00`
- **Frame Footer:** `0x55 0xCC`
- **Target Payload Block:** 8 bytes per target (supports up to 3 simultaneous targets):
  - Bytes `0..1`: X position (sign-magnitude mm)
  - Bytes `2..3`: Y position (sign-magnitude mm)
  - Bytes `4..5`: Radial speed (sign-magnitude cm/s)
  - Bytes `6..7`: Spatial resolution (mm)

**Sign-Magnitude Value Decoding:**
```cpp
int16_t val = ((high & 0x7F) << 8) | low;
if ((high & 0x80) == 0) {
    val = -val;
}
```

---

## Fall Verification State Machine

```mermaid
stateDiagram-v2
    [*] --> Monitor : Power On / Reset

    state Monitor {
        [*] --> ContinuousKinematics
        ContinuousKinematics : 50Hz Real-Time IMU Stream
        ContinuousKinematics : On-Device ML Classifier Inference
    }

    Monitor --> CheckStillness : Impact > 2.5g OR ML Fall Score > 85%

    state CheckStillness {
        [*] --> WindowEvaluation
        WindowEvaluation : 3-Second Window (25 samples)
        WindowEvaluation : Measure Stillness (|a| - 1.0g < 0.25g)
    }

    CheckStillness --> Monitor : Movement Detected (Self-Recovery / False Alarm)
    CheckStillness --> ConfirmedFall : Continuous Stillness Confirmed

    state ConfirmedFall {
        [*] --> EmergencyResponse
        EmergencyResponse : Energize GP16 Optocoupler Relay (Power ESP32-CAM)
        EmergencyResponse : Trigger CrowPanel Tactical Audio-Visual Alarm
        EmergencyResponse : Dispatch Telegram Bot Alert with Multi-Frame Snapshots
    }
```
