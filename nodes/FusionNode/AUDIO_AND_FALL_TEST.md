# Fusion Pico v5.2 audio, buzzer and fall bench test

This applies to experimental firmware `5.2-FUSION-AUDIO-FALL`. It is compatible
with the known-working CrowPanel v5.5 protocol v2 image.

## Wiring to verify before power-on

| Device | Pico connection |
| --- | --- |
| Passive piezo/beeper signal | GP22 |
| Piezo ground | GND |
| DFPlayer RX | GP8 (Pico TX; use the existing recommended series resistor) |
| DFPlayer TX | GP9 (Pico RX; optional but enables finish/error feedback) |
| DFPlayer serial ground | Pico GND |

The code defaults to a passive piezo driven by `tone()`. If the attached module
is an active buzzer that expects steady DC, change only
`BUZZER_IS_ACTIVE = true`, rebuild under a new experimental filename, and log it
in `CHANGES.md`.

Do not power a speaker directly from a Pico GPIO. The buzzer/piezo input must be
within the GPIO's safe load; use the existing driver/transistor if the device
draws meaningful current.

## DFPlayer microSD layout

Use a FAT32 microSD card and this exact logical mapping:

```text
/MP3/0001.mp3  boot
/MP3/0002.mp3  wearable connected
/MP3/0003.mp3  wearable disconnected
/MP3/0004.mp3  fall alert
/MP3/0005.mp3  system ready
```

`playMp3Folder()` addresses these four-digit names. If the spoken meanings do
not match, correct the files/names on the card or the five centralized
`AudioTrack` values; do not scatter track numbers through event logic.

## Isolated startup tests

Open the Pico USB serial port at 115200 and reboot.

Expected:

1. Version banner contains `5.2-FUSION-AUDIO-FALL`.
2. A two-tone GP22 self-test sounds for about 1.8 seconds.
3. Log shows boot and ready clips queued.
4. Track 1 completes before track 5 starts; neither is started by a fixed
   700 ms delay.

Serial test keys:

```text
B       repeat GP22 buzzer self-test
1..5    queue that exact MP3 track
H or ?  print help
```

If `B` logs correctly but there is silence, first confirm GP22 versus physical
pin 29, common ground, buzzer polarity/type and whether a driver transistor is
required. GP14 is the green RGB status output in this firmware; it is not the
buzzer pin.

## Controlled fall test

Do not test by letting a person fall. Secure the wearable in a padded object.

1. Confirm Pico logs increasing `imu=` counts.
2. Produce a brief acceleration peak above 2.5 g.
3. Place the wearable still immediately afterward for at least 3 seconds.
4. Expect logs for candidate source, stillness start and confirmation.
5. CrowPanel should receive `fallState=2`, show the alert, and activate the
   existing alarm path. Pico GP22 repeats its pattern until cancellation.
6. Relay GP16 activates for 60 seconds.
7. Tap CrowPanel Cancel. Pico must stop buzzer/audio and return fall state to 0.

Movement during the stillness period resets the 3-second timer. Failure to
achieve continuous stillness within 8 seconds rejects the candidate. A stale
IMU stream cannot confirm it.

Also test an ordinary bump followed by continued movement. It should enter
candidate state temporarily and then reject rather than confirm.

## What to capture if anything is wrong

Save the Pico serial output from boot through the failure, including `[AUDIO]`,
`[BUZZER]`, `[FALL]`, `[BASE]` and `[STATS]` lines. Report which physical pin and
buzzer type are used, and list the exact microSD filenames. That evidence is
enough to adjust the next build without guessing.
