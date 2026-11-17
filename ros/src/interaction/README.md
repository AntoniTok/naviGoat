# Interaction

## Handle

- Topic: `/handle/vibration` using `std_msgs/msg/UInt8MultiArray`.
- Data: `[side, magnitude]`, where side is `0` for left or `1` for right and magnitude is 0 to 255. For example, `[1, 180]` vibrates the right side.
- Serial output: `<r:180>\n` at 115200 baud for that example. Each message sends one frame.
- Topic: `/handle/held` using `std_msgs/msg/Bool`. Serial input `<t:1>` publishes `true` and `<t:0>` publishes `false`.
- Device: `/dev/handle` inside the container, mapped from the ESP32 at `/dev/ttyUSB0` on the Jetson.

From the repository root:

```bash
docker run --rm -it --name navigoat-dev --network=host --device=/dev/ttyUSB0:/dev/handle -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Inside the container, from `/workspace`:

```bash
colcon build --packages-select handle_nodes --symlink-install
source install/setup.bash
ros2 launch handle_nodes handle_launch.py
```

## Wristband

- Topic: `/wristband/vibration` using `std_msgs/msg/UInt8MultiArray`.
- Data: `[side, magnitude]`, where side is `0` for left, `1` for right, `2` for forward or `3` for back and magnitude is 0 to 255.
- BLE output: `<l:180>`, `<r:180>`, `<f:180>` or `<b:180>`. Each message sends one frame.
- ESP32 BLE device name: `naviGoatWristband`.
- BLE service UUID: `0bbbe0d0-3a64-4f83-bc35-64314a26cb60`.
- BLE write characteristic UUID: `0bbbe0d1-3a64-4f83-bc35-64314a26cb60`.
- ESP32-C3 Super Mini PlatformIO firmware, pin mapping and standalone motor test:
  [firmware/wristband](../../../firmware/wristband/README.md).
- Refresh each active motor within 2 seconds to keep it running; magnitude 0
  stops that motor immediately. All motors stop on BLE disconnect.

Mount the host Bluetooth D-Bus socket when starting the container:

```bash
docker run --rm -it --name navigoat-dev --network=host -v /run/dbus:/run/dbus -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Inside the container, from `/workspace`:

```bash
colcon build --packages-select wristband_nodes --symlink-install
source install/setup.bash
ros2 launch wristband_nodes wristband_launch.py
```
