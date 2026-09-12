# AirSPlay32 v1.0.0-beta.1

## Features

- AirPlay 2 receiver
- ESP32-S3 N16R8 support
- PCM5102A I2S audio output
- Wi-Fi provisioning and web UI
- OTA firmware update support
- 5-band software EQ with immutable factory presets and saved Custom profile
- Runtime diagnostics for boot, reset reason, RTP discontinuities, and audio output

## Fixes

- Improved RTP timeline discontinuity handling around seek/resume.
- Reduced YouTube seek/resume mute risk by resetting timing continuity on new anchors and large RTP jumps.
- Improved audio output stability diagnostics without changing the validated I2S/DAC configuration.

## Known Beta Limitations

- Manual long-run beta validation is still pending for the exact target device after this build.
