# ros_botix_course

ROS 2 Jazzy workspace for the [Botix](https://github.com/KiraFlux/Botix) ESP32
rover. One node bridges the robot's two UDP streams onto the ROS graph.

| Direction | Topic | Type | Source |
| :-- | :-- | :-- | :-- |
| out | `/scan` | `sensor_msgs/LaserScan` | D500 lidar frames forwarded by the firmware |
| out | `/odom` | `nav_msgs/Odometry` | encoder counts, integrated on this side |
| out | `/joint_states` | `sensor_msgs/JointState` | encoder counts |
| out | `tf` | `odom` → `base_link` | same integration |
| in | `/cmd_vel` | `geometry_msgs/Twist` | converted to MAVLink `MANUAL_CONTROL` |

The robot needs firmware from the `feat/lidar-odometry` branch of
[botix-esp32-firmware](https://github.com/KiraFlux/botix-esp32-firmware): the
lidar forwarder and the encoder-tick messages live there.

## Wiring the lidar

The D500 transmits and never listens, so it needs one signal wire and a shared
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

The frame decoder has unit tests covering split frames, resynchronisation after
garbage, CRC rejection, a `0x54` inside a payload, datagram loss, and foreign
traffic on the port:

```bash
cd ros2_ws/src/botix_driver && python3 -m pytest test -q
```

The whole node has been exercised against a loopback stand-in that speaks both
streams: `/scan` published at 10.5 Hz, `/odom` integrated a straight 11.6 m run
to within rounding, and `/cmd_vel` at 0.25 m/s and 1.5 rad/s arrived as
`z=500, r=-500`.

**Not tested against the sensor.** The D500 was still wired to the wrong pin
when this was written, so the framing constants come from the LD06 family
datasheet rather than from captured bytes. If `/scan` stays empty while the
robot's `lidar` command shows bytes arriving, the decoder is the place to look:
the CRC polynomial and the 47-byte layout are the two things most likely to
differ on a clone.
