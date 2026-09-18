# RANGER pinout guide

This guide covers the active protocol-v3 stack. Verify the exact board revision and voltage requirements before wiring.

## WearableNode — Seeed XIAO nRF52840

| Function | XIAO pin | Notes |
| --- | --- | --- |
| I2C SDA | D4 | MPU6050 and optional environment sensors |
| I2C SCL | D5 | I2C clock |
| USB | USB-C | Programming, power and serial diagnostics |

The current target is the basic XIAO nRF52840. Seeed's shared UF2 bootloader may nevertheless mount with a `XIAO-SENSE` volume name.

Do not power a display or another high-current peripheral through the XIAO's onboard regulator.

## BaseNode — Seeed XIAO nRF52840

| Function | XIAO pin | Connects to |
| --- | --- | --- |
| UART TX | D6 | Fusion GP1 (RX) |
| UART RX | D7 | Fusion GP0 (TX) |
| Ground | GND | Fusion GND |

UART operates at 115200 baud, 8N1.

## FusionNode — Raspberry Pi Pico / RP2040

| Function | Pico GPIO | Connects to / notes |
| --- | --- | --- |
| Base UART TX | GP0 | Base D7 (RX) |
| Base UART RX | GP1 | Base D6 (TX) |
| I2C1 SDA | GP2 | CrowPanel GPIO15 |
| I2C1 SCL | GP3 | CrowPanel GPIO16 |
| Radar UART TX | GP4 | RD-03D RX |
| Radar UART RX | GP5 | RD-03D TX |
| DFPlayer TX | GP8 | DFPlayer RX, normally through 1 kΩ |
| DFPlayer RX | GP9 | DFPlayer TX |
| RGB red | GP13 | Active-high status output |
| RGB green | GP14 | Active-high status output |
| RGB blue | GP15 | Active-high status output |
| Camera relay | GP16 | Optoisolated relay input, active high |
| Local buzzer | GP22 | Tone/PWM or steady output according to buzzer type |

The RD-03D UART operates at 256000 baud. Fusion exposes `Wire1` as I2C slave address `0x42`.

## CrowPanel Advance 4.3-inch V1.0

| Function | ESP32-S3 GPIO / address | Notes |
| --- | --- | --- |
| Shared I2C SDA | GPIO15 | Touch, expander and Fusion GP2 |
| Shared I2C SCL | GPIO16 | Touch, expander and Fusion GP3 |
| Touch interrupt | GPIO1 | GT911 interrupt/control during initialization |
| Passive buzzer | GPIO8 | Direct GPIO tone output |
| GT911 touch | I2C `0x5D` | Capacitive touch controller |
| TCA9534-compatible expander | I2C `0x18` | Backlight and touch reset control |
| Fusion Pico | I2C `0x42` | Ranger Link v3 telemetry and commands |

GPIO6 is used by the board's audio interface and is **not** an available user button.

### RGB display bus

```text
D0..D15 = 21,47,48,45,38,9,10,11,12,13,14,7,17,18,3,46
DE      = 42
VSYNC   = 41
HSYNC   = 40
PCLK    = 39
```

The tested ST7265 timing uses a 21 MHz pixel clock with horizontal and vertical front porch 8, pulse width 4 and back porch 8. PSRAM must be configured as 8 MB OPI.

## CrowPanel-to-Pico wiring

When both devices have their own USB power:

| CrowPanel | Pico | Signal |
| --- | --- | --- |
| GPIO15 | GP2 | SDA |
| GPIO16 | GP3 | SCL |
| GND | GND | Common reference |

Use short wires, preferably under 20 cm. The tested bus rate is 100 kHz because the link shares CrowPanel's I2C bus with touch and the I/O expander.

Do not connect the two boards' 3.3 V or 5 V rails when they are separately powered.

## RD-03D radar

Cross UART directions:

| RD-03D | Fusion Pico |
| --- | --- |
| TX | GP5 (RX) |
| RX | GP4 (TX) |
| GND | GND |

Confirm the module's required supply voltage from its board documentation. UART logic and power requirements are separate concerns.

## Optional camera relay

Fusion GP16 should drive the input side of an appropriate optoisolated relay or driver. Do not power a camera directly from a GPIO. The relay controls the camera's suitable external supply.

## Power checklist

- Establish a common signal ground where required.
- Do not parallel independently powered voltage rails.
- Confirm 3.3 V logic compatibility before connecting UART or I2C.
- Keep display and camera current off small MCU regulator outputs.
- Disconnect power immediately if a regulator, MCU or battery becomes abnormally hot.
