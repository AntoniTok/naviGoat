# ESP32-C3 Super Mini wristband

PlatformIO firmware for `ros/src/interaction/wristband_nodes`. The default
environment runs a BLE peripheral called `naviGoatWristband` and controls four
motor-driver inputs using 20 kHz, 8-bit PWM. No micro-ROS agent is needed.

## Wiring

The pin order follows the supplied motor test. Change `kMotorPins` in
[`include/wristband_config.h`](include/wristband_config.h) if the motors are
mounted in a different order.

| ROS side | Direction | Driver input GPIO |
| --- | --- | --- |
| 0 | Left | 0 |
| 1 | Right | 1 |
| 2 | Forward | 20 |
| 3 | Back | 21 |

These are active-high motor-driver inputs, with a common ground between the
ESP32 and drivers. GPIO 20/21 are also UART0 RX/TX; the application leaves UART0
unused. USB CDC uses the board's native USB connection instead.

## Build and flash

Open this directory in PlatformIO, or run from the repository root:

```sh
pio run -d firmware/wristband
pio run -d firmware/wristband -t upload
```

Add `--upload-port COM5` (Windows) or `--upload-port /dev/ttyACM0` (Linux) if
automatic port detection chooses the wrong device. For initial flashing, if the
board is not detected, hold BOOT, press and release RESET, then release BOOT and
retry the upload. Reset the board after flashing if it stays in download mode.

The project pins Espressif32 6.13.0 / Arduino-ESP32 2.0.17 and uses PlatformIO's
`esp32-c3-devkitm-1` target for the Super Mini's ESP32-C3 with 4 MB flash. Flash
mode is DIO, and native USB CDC is enabled. Board reference:
[PlatformIO ESP32-C3-DevKitM-1](https://docs.platformio.org/en/latest/boards/espressif32/esp32-c3-devkitm-1.html).

## BLE protocol

- Service UUID: `0bbbe0d0-3a64-4f83-bc35-64314a26cb60`
- Write characteristic UUID: `0bbbe0d1-3a64-4f83-bc35-64314a26cb60`
- Supports writes with or without response.
- Send one complete ASCII frame per write: `<l:180>`, `<r:180>`, `<f:180>`,
  or `<b:180>`. Do not include a newline.
- Magnitude is 0 (off) through 255 (full power). Each command changes only its
  selected motor; multiple motors can run together. Invalid frames are ignored.
- All outputs start off and stop when BLE disconnects. Advertising resumes so
  the ROS bridge can reconnect.
- Each motor also stops if it receives no valid command for **2 seconds**, even
  if BLE stays connected. Refresh each active motor, for example at 10 Hz, for
  continuous vibration. Change `kCommandTimeoutMs` in the configuration header;
  `0` disables expiry and holds each value until a stop command or disconnect.

The ROS bridge uses the same characteristic UUID and retains the newest pending
command for each side, including stop commands when switching direction.

## Test from ROS

Build and launch `wristband_nodes` as described in
[`ros/src/interaction/README.md`](../../ros/src/interaction/README.md), then run:

```sh
# Left motor at magnitude 180; Ctrl+C stops publishing, then the watchdog stops it.
ros2 topic pub -r 10 /wristband/vibration std_msgs/msg/UInt8MultiArray "{data: [0, 180]}"

# Explicitly stop the left motor.
ros2 topic pub --once /wristband/vibration std_msgs/msg/UInt8MultiArray "{data: [0, 0]}"
```

## Standalone motor test

The `motor-test` environment reproduces the supplied sketch: GPIO 0, 1, 20, then
21 turn on for 1 second each, with 500 ms off between motors and a 2 second pause
after the sequence. This environment does not start BLE.

```sh
pio run -d firmware/wristband -e motor-test -t upload
```

Upload the default environment again to restore ROS/BLE control:

```sh
pio run -d firmware/wristband -e esp32-c3-supermini -t upload
```
