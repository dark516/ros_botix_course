# Botix SLAM and Navigation Design

## Goal

Add a staged ROS 2 Jazzy workflow for the physical Botix rover: first build and
save a map under manual control, then restart against that saved map and drive
autonomously with Navigation2. The existing ESP32 bridge remains the sole
hardware interface.

## Package Boundary

Create one `ament_python` package named `botix_navigation`. It owns SLAM,
localization, Nav2, command arbitration, maps, navigation RViz configuration,
and launch composition. `botix_driver` continues to own UDP, `/scan`, `/odom`,
`/joint_states`, `odom -> base_footprint`, and the physical `/cmd_vel` input.
`botix_description` continues to own robot geometry and fixed transforms.

The navigation package must not introduce `ros2_control`, a second odometry
publisher, or another `odom -> base_footprint` transform.

## Operating Modes

### Mapping

`mapping.launch.py` starts the existing hardware bringup without its RViz,
`twist_mux`, asynchronous SLAM Toolbox, and a mapping RViz profile. Manual
teleoperation publishes to `/cmd_vel_teleop`. SLAM Toolbox consumes `/scan` and
the existing odometry TF and publishes `/map` plus `map -> odom`.

The operator saves a completed map through the standard SLAM Toolbox serialize
service. The package provides a command-line helper which writes map assets
under a caller-selected path; generated maps are not committed automatically.

### Navigation

`navigation.launch.py` requires a map YAML path. It starts hardware bringup
without its RViz, `twist_mux`, Nav2 localization, navigation servers, and a Nav2
RViz profile. AMCL owns `map -> odom`; SLAM Toolbox is absent. Nav2 velocity is
remapped to `/cmd_vel_nav`, and only `twist_mux` publishes physical `/cmd_vel`.

Manual teleoperation remains available at higher priority than Nav2 so an
operator command overrides autonomous motion. A mux lock topic can inhibit all
motion, and the ESP32/bridge command timeout remains the final stop layer.

## Topic and TF Contracts

Inputs already supplied by hardware:

- `/scan` (`sensor_msgs/LaserScan`), frame `laser_frame`, best-effort.
- `/odom` (`nav_msgs/Odometry`), parent `odom`, child `base_footprint`.
- `/joint_states` and `/robot_description`.
- TF `odom -> base_footprint -> base_link -> laser_frame`.

Command arbitration:

- `/cmd_vel_teleop` (`geometry_msgs/Twist`), priority 100, timeout 0.5 s.
- `/cmd_vel_nav` (`geometry_msgs/Twist`), priority 50, timeout 0.5 s.
- `/cmd_vel_lock` (`std_msgs/Bool`), priority 255.
- `/cmd_vel` is published only by `twist_mux` and consumed by `botix_driver`.

Mapping adds `/map` and one `map -> odom` publisher from SLAM Toolbox.
Navigation adds `/map` from map_server and one `map -> odom` publisher from
AMCL. The two modes must never start both transform authorities.

## Geometry and Safety Parameters

Use a polygon footprint derived conservatively from the current chassis:
`[[0.10, 0.10], [0.10, -0.10], [-0.10, -0.10], [-0.10, 0.10]]` metres.
Both local and global costmaps consume `/scan` as a 2-D obstacle and clearing
source. The local costmap rolls in `odom`; the global costmap uses `map`.

Initial autonomous limits are deliberately below the manually tested range:

- maximum linear velocity: 0.18 m/s;
- maximum angular velocity: 0.8 rad/s;
- acceleration/deceleration limited to avoid wheel slip;
- robot radius is not used alongside the polygon footprint;
- transform tolerance accounts for the approximately 6 Hz lidar.

The initial stack uses NavFn for global planning and Regulated Pure Pursuit for
local control. It enables progress checking, goal checking, clearing, spin,
backup, and wait recoveries. Collision Monitor is outside this first stage;
the costmaps and command mux are the active safety layers.

## Localization and Sensor Fusion

Mapping uses wheel odometry directly. Navigation uses AMCL over the saved map
and lidar, while wheel odometry supplies short-term motion. `robot_localization`
is installed but not launched because no IMU is currently available; adding an
EKF with wheel odometry alone would add latency without an independent motion
measurement. IMU integration is a separate follow-up.

The bridge must eventually publish measured odometry covariance. This stage
adds conservative static covariance values suitable for AMCL/Nav2 rather than
leaving every covariance element zero.

## Configuration and Launch Structure

`botix_navigation` contains:

- `config/slam_toolbox.yaml` for asynchronous mapping;
- `config/nav2.yaml` for AMCL, planner, controller, costmaps, behavior server,
  velocity smoother, BT navigator, waypoint follower, and lifecycle managers;
- `config/twist_mux.yaml` for command priorities and lock;
- `launch/mapping.launch.py` and `launch/navigation.launch.py`;
- `rviz/mapping.rviz` and `rviz/navigation.rviz`;
- `botix_navigation/save_map.py` as the map serialization CLI;
- `maps/.gitkeep` as the default local map directory.

Launch arguments include `robot_host`, `params_file` overrides, `use_rviz`, and
for navigation a mandatory `map` path. Missing or unreadable map files fail
before starting autonomous servers.

## Failure Handling

- A missing map prevents navigation launch with a clear error.
- A stale teleop or Nav2 input expires in `twist_mux` after 0.5 seconds.
- A lock forces zero output regardless of command source.
- If SLAM or Nav2 lifecycle activation fails, autonomous commands are absent;
  the bridge timeout stops the motors.
- Mapping and navigation launch descriptions do not run together under the same
  namespace.
- Map save failures return a nonzero process status and preserve existing map
  files.

## Verification

Static tests validate package metadata, YAML parsing, required node sections,
topic remappings, mode-exclusive `map -> odom` ownership, map path validation,
and covariance values. The full workspace must pass `colcon build`, `colcon
test`, and `colcon test-result --verbose`.

Live mapping acceptance requires:

- `/scan`, `/odom`, `/tf`, and `/joint_states` continue at observed rates;
- `/map` publishes while manually driving via `/cmd_vel_teleop`;
- one connected TF tree contains `map -> odom -> base_footprint`;
- lidar points align consistently after a closed loop;
- map serialization produces loadable map files.

Live navigation acceptance requires:

- the saved map loads after a clean restart;
- AMCL publishes `map -> odom` and converges after an initial pose;
- a short RViz goal produces `/cmd_vel_nav`, then physical `/cmd_vel` through
  the mux;
- teleop overrides Nav2 and the lock suppresses both;
- cancel, stale commands, and process shutdown leave zero wheel command.

## Deferred Work

IMU fusion, automatic docking, Collision Monitor zones, keepout/speed filters,
multi-floor maps, and production-grade autonomous calibration are deliberately
deferred until the first physical map and navigation run are stable.
