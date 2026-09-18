# RANGER

RANGER is a local, multi-node fall-detection and room-monitoring prototype. It combines wearable TinyML inference, a dedicated BLE bridge, an RP2040 coordination node, an RD-03D mmWave radar, and an ESP32-S3 touch display.

The active system performs inference and alarm handling locally. It does not require a cloud service for fall detection, and the radar provides contactless presence/range information without recording identifiable video.

> This is an educational prototype, not a certified medical or emergency-response device.

## Current working stack

| Component | Hardware | Firmware | Role |
| --- | --- | --- | --- |
| Wearable | Seeed XIAO nRF52840 (basic model) | `4.1-WEAR-RIGID-ML` | MPU6050 sampling and Edge Impulse inference |
| Base | Seeed XIAO nRF52840 | `4.1-BASE-RIGID-BRIDGE` | BLE central and UART bridge |
| Fusion | Raspberry Pi Pico / RP2040 | `5.3-FUSION-FALL-CONFIG` | Fall policy, radar parsing, alarm outputs and I2C telemetry |
| Display | CrowPanel Advance 4.3-inch V1.0 / ESP32-S3 | `5.8-CROW-DIRECT-ALERTS` | Touch UI, settings, alarm presentation and optional Telegram text |

CrowPanel and Fusion use **Ranger Link protocol v3**. Their copies of `RangerLinkProtocol.h` must remain byte-for-byte identical and should be upgraded as a matched pair.

## Architecture

```text
Wearable (nRF52840 + MPU6050 + Edge Impulse)
    | BLE notifications: IMU / environment / ML scores
    v
Base (nRF52840 BLE central)
    | UART CSV, 115200 baud
    v
Fusion Pico (RP2040) <--- RD-03D radar, 256000 baud
    | Ranger Link v3, I2C slave 0x42
    v
CrowPanel (ESP32-S3, 800x480 touch display)

Fusion GP16 ---> optoisolated relay ---> optional ESP32-CAM power
```

The active CrowPanel/Pico path does not use ESP-NOW. The ESP-NOW sketches retained in the repository are older experimental nodes and are not required by the current stack.

## Fall-detection policy

The wearable runs a four-class Edge Impulse model with the deployed labels:

- fall
- false alarm
- idle
- walking

The Fusion node treats a result as a possible fall only when the fall class:

1. reaches the configured threshold;
2. wins over the other classes; and
3. leads the false-alarm class by the required margin.

Post-event stillness confirmation and the raw-impact fallback are independently configurable from the CrowPanel. Current upgrade-safe defaults are:

- impact fallback: **off**
- stillness confirmation: **on**

The RD-03D supplies target count, range and speed to the active Pico/CrowPanel stack. It is deliberately not a mandatory fall gate in the current rigid ML policy. This permits wearable-only operation and avoids environmental radar motion influencing a trained wearable decision.

## Repository layout

```text
nodes/
  WearableNode/       Current XIAO nRF52840 wearable
  BaseNode/           Current XIAO nRF52840 BLE bridge
  FusionNode/         Current RP2040 coordination firmware
  CrowPanel/          Current ESP32-S3 display firmware
  ESPCAM_Telegram/    Optional emergency camera node
  WatchNode_S3/       Experimental ESP32-S3 wearable
  BathroomNode/       Experimental room radar node

legacy_ranger2/       Earlier standalone ESP32-S3 generation
tools/RadarOverlay/   Web Serial radar/telemetry diagnostic console
tools/EI_DataCollector/
tests/                Host-side protocol and fall-policy tests
docs/                 Architecture, wiring and technical notes
NRF54_Test/           Experimental nRF54L15 bring-up
```

## Communications

### Wearable to Base: BLE notifications

The wearable publishes fixed-size IMU, environment and ML-result packets. The Base validates the exact packet lengths, decodes multi-byte fields explicitly, tracks malformed packets, and forwards accepted messages over UART.

### Base to Fusion: UART

The Base produces debuggable newline-delimited records at 115200 baud:

```text
I,ax,ay,az,gx,gy,gz,sequence
E,pressure,ir,red,battery,sos,sequence
M,fall,false_alarm,idle,walking,winner,confidence,flags
STAT,BASE_LINK_OK
STAT,W_LOST
```

### Radar to Fusion: UART

The RD-03D sends binary multi-target frames at 256000 baud. Fusion validates the frame markers, decodes signed X/Y position and radial speed, rejects invalid targets and publishes the selected target state.

### Fusion to CrowPanel: I2C

CrowPanel is the I2C master. Fusion uses `Wire1` as slave address `0x42`:

| CrowPanel | Pico | Signal |
| --- | --- | --- |
| GPIO15 | GP2 | SDA |
| GPIO16 | GP3 | SCL |
| GND | GND | Common reference |

Both boards should use their own appropriate power supply. Do not join their 3.3 V or 5 V rails simply because the I2C signals are connected.

Ranger Link v3 uses a packed 32-byte telemetry packet, a monotonically increasing sequence number, CRC-8 validation, stale/frozen-link detection and small validated command packets.

## CrowPanel build configuration

The tested panel is the Elecrow CrowPanel Advance 4.3-inch V1.0 with an ESP32-S3-WROOM-1-N16R8.

Arduino ESP32 core configuration:

```text
Board: ESP32S3 Dev Module
PSRAM: OPI PSRAM
Flash mode: QIO
Flash size: 16 MB
Partition: Huge APP
Serial monitor: 115200 baud
```

The display configuration uses the board's 800x480 ST7265 RGB panel, GT911 touch controller, 21 MHz pixel clock and 8 MB OPI PSRAM. Runtime pages use direct/event-driven drawing and bounded update regions to avoid RGB DMA/PSRAM contention. Continuous Wi-Fi and full-screen animation are intentionally avoided.

Before compiling CrowPanel:

```sh
cp nodes/CrowPanel/secrets.example.h nodes/CrowPanel/secrets.h
```

Fill the local `secrets.h` only if Telegram messaging is required. The file is ignored by Git and must never be committed.

## Nordic build notes

The current wearable and Base targets are the **basic Seeed XIAO nRF52840**, not the Sense model. The shared UF2 bootloader may still mount with a `XIAO-SENSE` volume label.

The wearable also requires the generated `falldetect_inferencing` Edge Impulse Arduino library matching the trained model.

## Tests

Run the host protocol and policy tests:

```sh
./tests/run_protocol_tests.sh
```

Run the radar parser fixtures:

```sh
node tools/RadarOverlay/tests/radar-parser.test.js
```

The protocol tests verify packet sizes, CRC behavior, command values, fall-policy gating and environment-field packing. The browser parser tests cover the legacy target format, compact target records, reordered telemetry, raw IMU/environment messages and status events.

## Radar diagnostic console

`tools/RadarOverlay/` contains a local Web Serial debugging interface with:

- signed X/forward-Y target plotting;
- target trails, range, speed and frame age;
- serial-rate and parser diagnostics;
- IMU, model and link telemetry;
- CSV export, camera overlay and a hardware-free demo stream.

On macOS, double-click `START_RADAR_VIEWER.command`, then connect from a Chromium-based browser. See the tool's own README for supported serial formats and limitations.

## Legacy Ranger2

`legacy_ranger2/` is the earlier standalone ESP32-S3 generation. Its fusion score combines wearable and radar evidence, but the wearable ML state remains its primary normal alarm trigger. It is retained for reproducibility and comparison; do not mix its protocol assumptions with the current Pico/CrowPanel stack.

## Safety and privacy

- Never commit Wi-Fi passwords, Telegram tokens, chat identifiers or device dumps.
- Do not power a display or another high-current peripheral through a XIAO's small onboard regulator.
- A visibly heating board should be disconnected and electrically inspected.
- Validate controlled falls using padding and a spotter; never test by creating an uncontrolled fall.
- Compilation and stationary inference output do not establish medical accuracy or classifier generalization.
