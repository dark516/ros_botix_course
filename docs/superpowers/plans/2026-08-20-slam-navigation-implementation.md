# Botix SLAM and Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add staged mapping, map saving, localization, and autonomous Navigation2 bringup for the physical Botix rover.

**Architecture:** A new `botix_navigation` package composes the existing hardware bringup with Twist Mux and either asynchronous SLAM Toolbox or Nav2 localization/navigation. The hardware bridge remains the only `/odom`, physical `/cmd_vel`, and `odom -> base_footprint` owner; SLAM Toolbox and AMCL own `map -> odom` in mutually exclusive modes.

**Tech Stack:** ROS 2 Jazzy, Python launch, SLAM Toolbox 2.8, Navigation2 1.3, Twist Mux 4.5, pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-08-20-slam-navigation-design.md`

## Global Constraints

- Wheel separation is `0.175 m`; wheel radius is `0.0335 m`.
- Lidar publishes best-effort `/scan` in `laser_frame` at approximately 6 Hz.
- `botix_driver` remains the only `/odom` and `odom -> base_footprint` publisher.
- Only `twist_mux` publishes physical `/cmd_vel` in mapping/navigation modes.
- Mapping and localization must never publish `map -> odom` simultaneously.
- Maximum autonomous speed is `0.18 m/s` linear and `0.8 rad/s` angular.
- Generated map assets are local runtime output and are not committed automatically.

---

### Task 1: Navigation Package Contract

**Files:**
- Create: `ros2_ws/src/botix_navigation/package.xml`
- Create: `ros2_ws/src/botix_navigation/setup.py`
- Create: `ros2_ws/src/botix_navigation/setup.cfg`
- Create: `ros2_ws/src/botix_navigation/resource/botix_navigation`
- Create: `ros2_ws/src/botix_navigation/botix_navigation/__init__.py`
- Create: `ros2_ws/src/botix_navigation/test/test_configuration.py`

**Interfaces:**
- Consumes: installed ROS packages `slam_toolbox`, `nav2_bringup`, `twist_mux`, and `botix_driver`.
- Produces: installable `botix_navigation` package and shared config/launch/map/rviz assets.

- [ ] Write failing tests that load every package YAML, assert the polygon footprint, velocity limits, mux priorities, and required Nav2 node sections.
- [ ] Run `pytest -q ros2_ws/src/botix_navigation/test/test_configuration.py` and verify failure because package assets do not exist.
- [ ] Add package metadata and data-file installation globs for config, launch, maps, and RViz.
- [ ] Re-run the focused test to reach the next missing configuration failure.

### Task 2: Twist Mux and Odometry Covariance

**Files:**
- Create: `ros2_ws/src/botix_navigation/config/twist_mux.yaml`
- Modify: `ros2_ws/src/botix_driver/botix_driver/bridge_node.py`
- Modify: `ros2_ws/src/botix_driver/config/botix.yaml`
- Modify: `ros2_ws/src/botix_driver/test/test_odometry.py`

**Interfaces:**
- Consumes: `/cmd_vel_teleop`, `/cmd_vel_nav`, `/cmd_vel_lock`.
- Produces: physical `/cmd_vel`; nonzero planar covariance on `/odom`.

- [ ] Add failing tests for covariance construction and mux config values.
- [ ] Implement a pure `planar_covariance()` helper and assign pose/twist covariance in the bridge.
- [ ] Configure teleop priority 100, Nav2 priority 50, both timeout 0.5 s, and lock priority 255.
- [ ] Run driver and navigation tests and verify green.

### Task 3: Mapping Configuration and Launch

**Files:**
- Create: `ros2_ws/src/botix_navigation/config/slam_toolbox.yaml`
- Create: `ros2_ws/src/botix_navigation/launch/mapping.launch.py`
- Create: `ros2_ws/src/botix_navigation/rviz/mapping.rviz`
- Extend: `ros2_ws/src/botix_navigation/test/test_configuration.py`

**Interfaces:**
- Consumes: `/scan`, `odom -> base_footprint`, `/cmd_vel_teleop`.
- Produces: `/map`, `map -> odom`, physical `/cmd_vel`, mapping RViz.

- [ ] Add failing assertions for asynchronous mapping mode, frames, scan topic, map resolution, and launch remappings.
- [ ] Configure SLAM Toolbox for `map`, `odom`, `base_footprint`, `/scan`, 0.05 m resolution, and lidar-rate-compatible throttling/tolerances.
- [ ] Compose hardware bringup with `use_rviz:=false`, Twist Mux, async SLAM Toolbox, and optional mapping RViz.
- [ ] Run focused tests and `ros2 launch ... --show-args`.

### Task 4: Map Serialization CLI

**Files:**
- Create: `ros2_ws/src/botix_navigation/botix_navigation/save_map.py`
- Create: `ros2_ws/src/botix_navigation/test/test_save_map.py`
- Modify: `ros2_ws/src/botix_navigation/setup.py`
- Create: `ros2_ws/src/botix_navigation/maps/.gitkeep`

**Interfaces:**
- Consumes: SLAM Toolbox `/slam_toolbox/serialize_map` service.
- Produces: `save_map` console command and serialized map path.

- [ ] Write failing unit tests for path validation, parent creation policy, existing-file refusal, and service result handling.
- [ ] Implement argument parsing and a ROS client for `slam_toolbox/srv/SerializePoseGraph`.
- [ ] Register `save_map = botix_navigation.save_map:main`.
- [ ] Run focused tests.

### Task 5: Navigation2 Configuration and Launch

**Files:**
- Create: `ros2_ws/src/botix_navigation/config/nav2.yaml`
- Create: `ros2_ws/src/botix_navigation/launch/navigation.launch.py`
- Create: `ros2_ws/src/botix_navigation/rviz/navigation.rviz`
- Extend: `ros2_ws/src/botix_navigation/test/test_configuration.py`

**Interfaces:**
- Consumes: map YAML, `/scan`, `/odom`, TF, `/cmd_vel_teleop`.
- Produces: AMCL `map -> odom`, `/cmd_vel_nav`, physical `/cmd_vel`, Nav2 actions and RViz goals.

- [ ] Add failing assertions for AMCL/Nav2 sections, frames, polygon footprint, obstacle layer, planners/controllers, speed limits, and cmd_vel remapping.
- [ ] Adapt the installed Jazzy Nav2 parameter schema with NavFn, Regulated Pure Pursuit, velocity smoother, recoveries, and lifecycle managers.
- [ ] Validate the map launch argument before including localization and navigation launch files.
- [ ] Remap Nav2 velocity output to `/cmd_vel_nav`; start Twist Mux and optional navigation RViz.
- [ ] Run focused tests and launch argument inspection.

### Task 6: Documentation and Static Integration

**Files:**
- Modify: `README.md`
- Create: `ros2_ws/src/botix_navigation/README.md`

**Interfaces:**
- Produces: exact operator workflows for mapping, saving, localization, teleop override, lock, and shutdown.

- [ ] Document package installation, build, mapping launch, teleop remap, map save command, navigation launch, initial pose, goal setting, and emergency lock commands.
- [ ] Run all pytest tests.
- [ ] Run `colcon build --symlink-install`, `colcon test`, and `colcon test-result --verbose`.
- [ ] Commit the complete static integration.

### Task 7: Live Mapping Acceptance

**Files:**
- Runtime output only under `ros2_ws/maps-test/`; do not commit generated maps.

**Interfaces:**
- Validates: hardware topics, mux routing, SLAM `/map`, TF ownership, serialization, RViz.

- [ ] Stop previous bringup instances and launch `mapping.launch.py` against `botix.local`.
- [ ] Verify `/scan`, `/odom`, `/map`, `/tf`, and `/joint_states` rates and one `map -> odom` authority.
- [ ] Send bounded `/cmd_vel_teleop` pulses, verify physical `/cmd_vel`, and finish with zero.
- [ ] Verify `/cmd_vel_lock` suppresses commands.
- [ ] Save a test map and verify generated files can be loaded.
- [ ] Leave mapping RViz open for the operator.

### Task 8: Publish

**Files:**
- Review all tracked changes; exclude generated frame graphs and maps.

**Interfaces:**
- Produces: pushed `ros_botix_course/main` containing the complete first navigation stage.

- [ ] Review type/topic/frame consistency and `git diff --check`.
- [ ] Run final tests after any review fixes.
- [ ] Commit remaining live-test/documentation adjustments.
- [ ] Push `main` and report commits, live results, and the exact mapping workflow.
