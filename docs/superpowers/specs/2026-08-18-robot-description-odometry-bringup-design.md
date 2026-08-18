# Botix Robot Description and Odometry Bringup

## Goal

Provide one hardware bringup for the current Botix rover that displays the live
robot model, Camsense lidar, wheel motion, encoder odometry, and TF in RViz. The
result must expose the standard ROS interfaces needed by a later SLAM Toolbox
and Nav2 integration, without claiming localization accuracy before odometry is
calibrated.

## Architecture

`botix_description` owns geometry, link names, joint names, and the lidar mount.
`botix_driver` owns hardware UDP transport, encoder conversion, wheel odometry,
and `cmd_vel`. A single hardware launch file in `botix_driver` loads the URDF
from `botix_description`, starts `robot_state_publisher`, starts the bridge, and
optionally starts RViz.

The live launch must not start `joint_state_publisher` or its GUI. Those nodes
generate synthetic wheel positions and would conflict with encoder-backed
`/joint_states`. The standalone description viewer may continue to offer them
for offline model inspection.

## ROS Interfaces

Published by the bridge:

- `/scan` (`sensor_msgs/LaserScan`), frame `laser_frame`.
- `/joint_states` (`sensor_msgs/JointState`), wheel positions in radians and
  wheel velocities in radians per second.
- `/odom` (`nav_msgs/Odometry`), parent `odom`, child `base_footprint`.
- dynamic TF `odom -> base_footprint`.

Published by `robot_state_publisher`:

- fixed TF `base_footprint -> base_link` and all rigid body frames.
- dynamic TF from `base_link` to both wheel links using `/joint_states`.
- `/robot_description` for RViz RobotModel.

Subscribed by the bridge:

- `/cmd_vel` (`geometry_msgs/Twist`) with the existing timeout-to-stop behavior.

The resulting tree is one connected hierarchy:

```text
odom
  -> base_footprint
    -> base_link
      -> left_wheel_link
      -> right_wheel_link
      -> laser_frame
      -> chassis links
```

## Kinematics

The URDF wheel radius is 0.0338 m and wheel separation is 0.1754 m. These become
the initial bridge defaults, with YAML parameters remaining authoritative for
calibration. Encoder distance is converted to wheel angle as
`distance_m / wheel_radius_m`; `JointState.position` must never contain metres.

Odometry continues to use differential-drive midpoint integration. Encoder
signs and `mm_per_tick` are read from parameters and verified against live
motion. The test sequence is stop, forward, reverse, positive yaw, and negative
yaw. Every command ends with zero velocity, including error paths.

## RViz

The hardware RViz profile uses `odom` as its fixed frame and enables:

- RobotModel from `/robot_description`.
- LaserScan from `/scan` using best-effort reliability.
- Odometry from `/odom` with a visible pose trail.
- TF and grid.

This is deliberately SLAM-style but has no `/map` yet. Adding a fake map frame
would conceal odometry faults. SLAM Toolbox will later provide `map -> odom`
after wheel scale, wheel separation, lidar pose, and scan orientation are
verified.

## Diagnostics and Acceptance

The integration is accepted only when a live test confirms:

- `/scan` publishes near the sensor's observed 6 Hz and contains finite ranges.
- both encoder values change with the expected signs for each motion.
- `/joint_states` and `/odom` publish while lidar continues publishing.
- wheel joint positions are radians and update the URDF wheels.
- TF has one connected tree with no duplicate parent for any frame.
- forward motion increases odometry X; in-place turns change yaw with the ROS
  sign convention; stopping leaves reported velocity at zero.
- RViz renders RobotModel, LaserScan, Odometry, and TF simultaneously without
  message-filter drops.

Unit tests cover encoder-to-wheel-angle conversion, differential-drive updates,
first-sample behavior, timestamps/velocities, and sign handling. The complete
workspace is built with `colcon`, followed by live topic-rate and TF checks.

## Navigation Boundary

This change prepares standard interfaces but does not add SLAM Toolbox, AMCL,
costmaps, planners, or controllers. The next stage is SLAM Toolbox mapping,
followed by localization and Nav2 only after odometry calibration passes.
