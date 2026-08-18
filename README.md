# ros_botix_course

ROS 2 Jazzy workspace for the [Botix](https://github.com/KiraFlux/Botix) ESP32
rover. One node bridges the robot's two UDP streams onto the ROS graph.

| Direction | Topic | Type | Source |
| :-- | :-- | :-- | :-- |
| out | `/scan` | `sensor_msgs/LaserScan` | Camsense X1 or LDROBOT frames forwarded by the firmware |
| out | `/odom` | `nav_msgs/Odometry` | encoder counts, integrated on this side |
| out | `/joint_states` | `sensor_msgs/JointState` | encoder counts |
| out | `tf` | `odom` → `base_link` | same integration |
| in | `/cmd_vel` | `geometry_msgs/Twist` | converted to MAVLink `MANUAL_CONTROL` |

The robot needs firmware from the `feat/lidar-odometry` branch of
[botix-esp32-firmware](https://github.com/KiraFlux/botix-esp32-firmware): the
lidar forwarder and the encoder-tick messages live there.

## Wiring the lidar

The installed sensor transmits and never listens, so it needs one signal wire and a shared
ground. **Do not use the pin silkscreened `RX`.** That is GPIO3, UART0 — the
console, the log and the programming line, already driven by the USB bridge.
A lidar there fights the bridge for the wire and feeds binary to the command
parser.

Use **GPIO16**, the RX of UART2. Leave GPIO17 unconnected.

## Setup

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
pip install pymavlink --break-system-packages
```

```bash
cd ros2_ws && colcon build && source install/setup.bash
```

Enable the lidar on the robot, over its console:

```bash
python3 ../botix-esp32-firmware/tools/botix_console.py --host botix.local shell
```

```
config set user.lidar.enabled true
config set user.lidar.uart 2
config set user.lidar.rx_pin 16
config set user.lidar.baudrate 115200
config save
reboot
```

`lidar` on that console reports bytes read and datagrams sent. Bytes climbing
while datagrams stay flat means the destination is wrong; both flat means
nothing is arriving on the wire, so check GPIO16 and the shared ground.

## Running

```bash
ros2 launch botix_driver bringup.launch.py robot_host:=botix.local
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
rviz2 -d rviz/botix.rviz
```

## Calibration

The defaults in `config/botix.yaml` are placeholders from one chassis. Odometry
means nothing until these match your robot:

- **`wheel_base`** — distance between the wheel contact patches. Command a
  known rotation and scale until the reported yaw matches.
- **`mm_per_tick`** — must equal `device.encoder.mm_per_tick` on the robot.
  Drive a measured straight line and compare against `/odom`.
- **`left_ticks_sign`** — the left motor is mounted mirrored on this chassis,
  so its encoder counts down while driving forward. Confirm with `telemetry`
  on the robot console before trusting anything downstream.
- **`invert_turn`** — the tank mixer computes `left = z + r`, turning clockwise
  for positive `r`, while positive `angular.z` in ROS is counter-clockwise.

The robot's own drivetrain trim is separate and lives in its config as
`device.mixer.left_scale` / `right_scale`.

## What has been tested

The frame decoder auto-detects Camsense X1 (`55 AA 03 08`, 36 bytes) and
LDROBOT (`54 2C`, 47 bytes). Its tests cover real captured Camsense data, split
frames, resynchronisation, checksum rejection, datagram loss, and foreign
traffic on the port:

```bash
cd ros2_ws/src/botix_driver && python3 -m pytest test -q
```

The installed lidar was verified over the complete GPIO16 -> ESP32 -> UDP ->
ROS path. It identifies as the Camsense X1 protocol rather than the advertised
D500/STL-19P protocol: 468 checksum-valid packets and 3744 points were decoded
in an 8-second probe. `/scan` publishes at about 6 Hz with measured ranges and
intensities. The loopback checks for odometry and `/cmd_vel` also pass.
