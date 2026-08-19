# Integrated Robot Odometry Bringup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Run the Botix model, calibrated ESP32 wheel odometry, lidar, TF, and `/cmd_vel` together in one ROS 2 Jazzy bringup and RViz session.

**Architecture:** Keep encoder calibration on the ESP32 and consume MAVLink `WHEEL_DISTANCE` directly in ROS. Isolate differential-drive math in a dependency-free Python module so metre/radian/sign behavior is unit-tested. Let `robot_state_publisher` own all URDF transforms while the bridge publishes only `odom -> base_footprint` and encoder-backed wheel joints.

**Tech Stack:** ROS 2 Jazzy, rclpy, pymavlink, robot_state_publisher, RViz2, pytest, colcon.

---

### Task 1: Test the calibrated wheel odometry contract

**Files:**
- Create: `ros2_ws/src/botix_driver/botix_driver/odometry.py`
- Create: `ros2_ws/src/botix_driver/test/test_odometry.py`

1. Add failing tests for first-sample behavior, straight travel, in-place rotation, midpoint arc integration, wheel angle in radians, and velocity from elapsed time.
2. Run `pytest -q ros2_ws/src/botix_driver/test/test_odometry.py` and confirm failures are caused by the missing odometry API.
3. Implement a minimal `DifferentialDriveOdometry` accepting cumulative left/right distances in metres, wheel separation, and wheel radius.
4. Re-run the focused test until green, then run all package tests.

### Task 2: Consume ESP-calibrated MAVLink distances

**Files:**
- Modify: `ros2_ws/src/botix_driver/botix_driver/bridge_node.py`
- Modify: `ros2_ws/src/botix_driver/config/botix.yaml`
- Modify: `ros2_ws/src/botix_driver/test/test_odometry.py`

1. Add a failing test for extracting the first two wheels from a `WHEEL_DISTANCE` sample and rejecting incomplete samples.
2. Refactor MAVLink handling so `WHEEL_DISTANCE` drives odometry; keep `NAMED_VALUE_INT` counters as diagnostics without integrating them.
3. Publish joint position and velocity in radians from calibrated wheel distances.
4. Set defaults to wheel separation `0.1754`, wheel radius `0.0338`, base frame `base_footprint`, and lidar frame `laser_frame`; remove ROS `mm_per_tick` and tick sign calibration.
5. Run focused and full package tests.

### Task 3: Integrate robot_description into hardware bringup

**Files:**
- Modify: `ros2_ws/src/botix_driver/launch/bringup.launch.py`
- Modify: `ros2_ws/src/botix_driver/package.xml`
- Modify: `ros2_ws/src/botix_driver/rviz/botix.rviz`
- Modify: `ros2_ws/src/botix_description/package.xml`
- Modify: `ros2_ws/src/botix_description/README.md`

1. Replace the duplicate static lidar transform with `robot_state_publisher` loading `botix_description/urdf/botix.urdf`.
2. Add a `use_rviz` argument and launch RViz with the hardware profile when requested.
3. Declare runtime dependencies between driver, description, state publisher, and RViz.
4. Configure RViz fixed frame `odom` and displays for RobotModel, LaserScan `/scan`, Odometry `/odom`, TF, and Grid.
5. Validate launch syntax and package metadata.

### Task 4: Build and static verification

**Files:**
- Verify all files under `ros2_ws/src/botix_driver` and `ros2_ws/src/botix_description`.

1. Source ROS Jazzy and run `colcon build --symlink-install` from `ros2_ws`.
2. Run `colcon test` and inspect `colcon test-result --verbose`.
3. Launch bringup without RViz briefly and verify nodes and topic types.
4. Inspect TF for one connected `odom -> base_footprint -> base_link -> laser_frame` tree and no duplicate publishers.

### Task 5: Live robot acceptance and RViz

**Files:**
- No source edits unless a live failure is reproduced by a new test first.

1. Confirm robot reachability and live `/scan` near 6 Hz with finite ranges.
2. Record `/odom`, `/joint_states`, and raw encoder telemetry at rest.
3. Send bounded forward, reverse, positive-yaw, and negative-yaw `/cmd_vel` pulses; send zero after every pulse and in cleanup.
4. Verify expected wheel-distance changes, forward X, yaw sign, joint radians, and lidar continuity during movement.
5. Launch the full bringup with RViz and leave the working RViz window open showing model, scan, odometry, TF, and grid.

### Task 6: Review and publish

**Files:**
- Review the complete git diff; exclude generated frame graphs and unrelated files.

1. Check type consistency across MAVLink metres, ROS metres, radians, timestamps, and frame IDs.
2. Confirm every design requirement has an implementation or an explicit navigation-stage boundary.
3. Run final tests and inspect git status.
4. Commit the ROS integration and push the current branch only after verification succeeds.
