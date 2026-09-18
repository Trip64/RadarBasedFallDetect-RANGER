# Current RANGER architecture

This document describes the protocol-v3 Pico/CrowPanel stack. Older sketches in `legacy_ranger2/`, `WatchNode_S3/` and `BathroomNode/` are retained for reference but are not part of this active data path.

## Data flow

```text
MPU6050 + Edge Impulse
        |
        v
WearableNode 4.1 (XIAO nRF52840, BLE peripheral)
        | fixed-size BLE notifications
        v
BaseNode 4.1 (XIAO nRF52840, BLE central)
        | UART CSV at 115200 baud
        v
FusionNode 5.3 (RP2040) <--- RD-03D UART at 256000 baud
        | I2C slave 0x42, Ranger Link v3
        v
CrowPanel 5.8 (ESP32-S3, I2C master and 800x480 UI)
```

Fusion can also operate an optoisolated camera-power relay, DFPlayer Mini, passive/active buzzer and RGB status LED.

## Responsibilities

### WearableNode

- Samples six-axis MPU6050 motion data.
- Runs the deployed four-class Edge Impulse model locally.
- Publishes IMU, environment and ML result packets over BLE.
- Validates sensor initialization and rejects failed reads.
- Uses overlapping inference windows to avoid blind boundaries between consecutive windows.

### BaseNode

- Scans for and connects to the wearable.
- Requires exact BLE characteristic lengths.
- Decodes multi-byte fields explicitly rather than relying on host alignment.
- Tracks malformed traffic and accepts any valid packet as link recovery.
- Converts BLE packets into readable UART records.

### FusionNode

- Parses Base UART messages and RD-03D binary frames.
- Enforces ML class, confidence and false-alarm-margin policy.
- Optionally enables raw-impact triggering.
- Optionally requires continuous post-event stillness.
- Latches confirmed alarms until an explicit validated cancel command.
- Publishes immutable, double-buffered I2C snapshots.
- Controls relay, RGB LED, buzzer and queued DFPlayer audio without blocking sensor processing.

### CrowPanel

- Polls Fusion at a bounded rate and verifies magic, protocol version and CRC-8.
- Detects stale packets, frozen sequence numbers and sequence gaps.
- Provides saved impact/stillness settings and re-synchronizes them after link acquisition.
- Draws runtime pages directly with bounded dirty regions.
- Keeps Wi-Fi off during ordinary monitoring.
- Uses Wi-Fi only for an optional emergency Telegram request, then turns it off.

The CrowPanel is a presentation/control layer. It is not the source of the wearable ML decision.

## Fall state machine

```text
NORMAL
  |
  | qualified ML result
  | or optional impact trigger
  v
CANDIDATE
  |-- stillness disabled ---------------------> CONFIRMED
  |-- stillness sustained for required time --> CONFIRMED
  `-- motion / timeout / settings change ------> NORMAL

CONFIRMED
  | alarm outputs and relay active
  `-- validated cancel command ---------------> NORMAL
```

The ML result must reach the configured threshold, win over the other model classes, and lead the false-alarm score by the configured margin. With stillness enabled, a stale IMU stream cannot confirm a candidate.

Radar data is reported for presence, target count, range and radial speed. It is not a mandatory fall-confirmation input in this version.

## Ranger Link v3

Ranger Link uses a packed 32-byte telemetry packet and a packed 5-byte command packet. Both protocol headers contain compile-time size assertions.

Telemetry includes:

- protocol magic and version;
- sequence number;
- acceleration and gyro data;
- fall state and wearable/radar/relay/SOS flags;
- battery percentage;
- ML fall and confidence values;
- radar range, speed and target count;
- configuration state; and
- CRC-8.

The sequence number differentiates a healthy stream from a peripheral that repeatedly returns an old but otherwise valid packet.

Commands include alarm cancellation, relay testing, impact-trigger configuration and stillness configuration. Every command is validated before it is queued for normal-loop processing. I2C interrupt callbacks only copy prepared bytes or queue flags; they do not perform logging, timing decisions or actuator work.

## Timing

| Operation | Nominal period |
| --- | --- |
| Fusion telemetry publication | 100 ms |
| CrowPanel Pico poll | 250 ms |
| CrowPanel dashboard refresh | 1 s |
| Motion graph transfer | 2 s |
| Touch poll | 25 ms |
| Link stale threshold | 1.5 s |

These rates intentionally separate communication from display rendering. The RGB panel scans a framebuffer in external memory, so repeated full-screen sprite transfers and continuous Wi-Fi activity are avoided.

## Failure handling

- Missing wearable: Base continues scanning; Fusion and CrowPanel expose link loss.
- Stale/frozen Pico telemetry: CrowPanel reports the link fault rather than displaying the packet as live.
- Invalid CRC/version/size: the packet is rejected.
- Missing radar: wearable ML remains available.
- Missing CrowPanel: Fusion continues processing its sensor inputs and local outputs.
- Configuration change during a candidate: Fusion rejects the in-progress candidate so its rules cannot change halfway through an event.

## Security and privacy boundary

Fall inference is local. The only optional internet path is the CrowPanel emergency Telegram request. Wi-Fi and Telegram values live in an ignored `secrets.h`; only `secrets.example.h` belongs in version control.
