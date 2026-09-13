# WLROBOT Remote Client (wl_bot)

Cross-platform remote for the Micro Wheeled-leg Robot: **Android / macOS / Windows**
from one Flutter codebase. Speaks the robot's LAN protocol directly
(WebSocket `:81` + JSON API `:80`, both cleartext).

## Features

- Drive screen: virtual joystick (0.15 dead zone), direction pad with
  momentary Jump button (firmware fires on the jump→stop edge), height/roll
  sliders, GO/STOP, live status + battery gauge + latency readout
- Physical gamepads (Xbox/PS layout): left stick, A=jump, B=stop,
  Start=GO toggle, D-pad, RT/LT = height. Keyboard works everywhere
  (WASD/arrows, Space=jump, X/Esc=stop).
- LAN discovery: /24 subnet scan with battery/state preview, tap to connect,
  plus remembered-host history.
- Settings: PID / LPF / balance zero / yaw / gyro cal / full jump profile /
  Wi-Fi OTA with reboot detection.

## Prerequisites

- Flutter SDK (tested with 3.47.x stable).
- Android builds reuse an existing SDK, e.g. the qnwl one:
  `flutter config --android-sdk ~/qnwl-android-sdk`
- Gradle needs JDK 17 (newer JDKs break the build):
  `flutter config --jdk-dir /opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home`
- macOS desktop builds need full Xcode + accepted license:
  `sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer`
  `sudo xcodebuild -license accept`
- Windows `.exe` must be built on a Windows PC (no cross-compile from macOS).

## Run / build

```bash
cd client/wl_bot
flutter pub get

# fast UI loop in the browser (talks to the real robot):
flutter run -d chrome

# Android device over adb:
flutter run -d <device-id>
flutter build apk --release        # sideload the APK

# macOS (needs Xcode, see above):
flutter run -d macos
```

## Tests

```bash
flutter analyze                      # must be clean
flutter test                         # unit + protocol tests
ROBOT_HOST=192.168.1.195 flutter test test/live_test.dart test/live_scan_test.dart
```

The `live_*` tests are opt-in (pass vacuously without `ROBOT_HOST`) so CI
stays green with no hardware.

## Protocol notes

- WS command JSON mirrors the firmware schema exactly:
  `{"mode":"basic","dir":...,"height":...,"roll":...,"linear":0,"angular":0,"stable":0|1,"joy_x":...,"joy_y":...}`
- Commands are throttled (40 ms, send-on-change + 250 ms heartbeat);
  status is polled every 250 ms (`test/protocol_test.dart` pins the schema).
- Firmware reference: `main/ws_server.c` (WS), `main/http_server.c` (REST).
