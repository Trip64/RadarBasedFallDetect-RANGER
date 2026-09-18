# RANGER Radar Viewer

This is a local diagnostic dashboard for the text debug output produced by the RANGER base controller. It does not change or flash any device.

## Quick start on macOS

1. Double-click `START_RADAR_VIEWER.command`.
2. In the Chrome page that opens, click **Connect serial**.
3. Choose the base controller's USB serial port.
4. Use **115200 baud** for the current RANGER firmware.

If macOS refuses to open the launcher, right-click it and choose **Open** once. Chrome or another Chromium browser is required for the serial connection. The **Start demo** button lets you check the entire dashboard without hardware.

## What it understands

- Legacy RD03 target lines such as `T1: x=-782mm y=1713mm spd=-16cm/s res=50mm VALID`
- Compact target lines such as `RADAR,1,-782,1713,-16,50,1`
- `TELEMETRY:` and `IMU:` key/value lines
- Fusion/base `I,...` raw IMU and `E,...` environment packets
- Watch-link status, fall-model events, and base statistics

The map deliberately shows the RD03's raw axes: signed X is left/right and positive Y is forward. It does not pretend those axes have been calibrated to the room. Target numbers are temporary radar slots, not persistent person identities.

The **256000 baud** choice is available for firmware that prints compatible text at that speed. The viewer does not decode a bare RD03 binary UART stream directly; connect it to the RANGER controller's debug serial output.

## Useful controls

- **Pause** freezes ingestion without closing the port.
- **Clear** removes target trails and the current CSV recording.
- **Save CSV** exports everything recorded since the last clear/reload.
- **Camera** places a low-opacity live camera feed behind the coordinate map.
- Shortcuts: `C` connect, `D` demo, `Space` pause, `X` clear.
