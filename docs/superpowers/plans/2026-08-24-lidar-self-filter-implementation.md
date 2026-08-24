# Botix Lidar Self-Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reusable `lidar_filter` ROS 2 package that removes only lidar endpoints inside the physical robot and routes its output through mapping and navigation.

**Architecture:** `botix_driver` publishes `/scan_raw` only when composed by navigation launch files. The standard `laser_filters/LaserScanBoxFilter` transforms endpoints into `base_footprint`, removes the configured self-box, and publishes the established `/scan` topic consumed by SLAM, AMCL, costmaps, and RViz.

**Tech Stack:** ROS 2 Jazzy, ament_python, Python launch, `laser_filters` 2.0.9, pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-08-24-lidar-self-filter-design.md`

## Global Constraints

- Do not add `LaserScanSpeckleFilter` or any other measurement filter.
- Preserve every range and intensity outside the self-box unchanged.
- Self-box in `base_footprint`: X `[-0.140, 0.140]`, Y `[-0.115, 0.115]`, Z `[0.000, 0.200]` metres.
- Standalone `botix_driver` keeps publishing `/scan` by default.
- Integrated mapping/navigation use `/scan_raw -> lidar_filter -> /scan`.
- No generated maps, bags, frame graphs, or cache files are committed.

---

### Task 1: Package and Filter Contract

**Files:**
- Create: `ros2_ws/src/lidar_filter/package.xml`
- Create: `ros2_ws/src/lidar_filter/setup.py`
- Create: `ros2_ws/src/lidar_filter/setup.cfg`
- Create: `ros2_ws/src/lidar_filter/resource/lidar_filter`
- Create: `ros2_ws/src/lidar_filter/lidar_filter/__init__.py`
- Create: `ros2_ws/src/lidar_filter/config/self_filter.yaml`
- Create: `ros2_ws/src/lidar_filter/test/test_configuration.py`

**Interfaces:**
- Consumes: `sensor_msgs/msg/LaserScan` on remappable `scan`.
- Produces: filtered `sensor_msgs/msg/LaserScan` on remappable `scan_filtered`.

- [x] Write a failing test that requires one filter named `self_box`, type `laser_filters/LaserScanBoxFilter`, `box_frame=base_footprint`, exact bounds, `invert=false`, and no `filter2`.
- [x] Run `pytest -q ros2_ws/src/lidar_filter/test/test_configuration.py` and verify it fails because the package config is absent.
- [x] Create package metadata with runtime dependencies on `laser_filters`, `launch`, and `launch_ros`, plus install rules for `config` and `launch`.
- [x] Add `self_filter.yaml` containing only the specified `LaserScanBoxFilter`.
- [x] Re-run the focused test and verify it passes.

### Task 2: Reusable Filter Launch

**Files:**
- Create: `ros2_ws/src/lidar_filter/launch/lidar_filter.launch.py`
- Extend: `ros2_ws/src/lidar_filter/test/test_configuration.py`

**Interfaces:**
- Consumes: launch arguments `input_scan` (default `/scan_raw`) and `output_scan` (default `/scan`).
- Produces: node `scan_to_scan_filter_chain` running the standard
  `laser_filters/scan_to_scan_filter_chain` executable.

- [x] Add a failing source-contract test requiring the two launch arguments, the standard executable, config path, and `scan`/`scan_filtered` remappings.
- [x] Implement the launch file with `LaunchConfiguration` remappings.
- [x] Run the package tests and `ros2 launch lidar_filter lidar_filter.launch.py --show-args`.

### Task 3: Driver and Navigation Routing

**Files:**
- Modify: `ros2_ws/src/botix_driver/launch/bringup.launch.py`
- Modify: `ros2_ws/src/botix_navigation/launch/mapping.launch.py`
- Modify: `ros2_ws/src/botix_navigation/launch/navigation.launch.py`
- Modify: `ros2_ws/src/botix_navigation/package.xml`
- Extend: `ros2_ws/src/lidar_filter/test/test_configuration.py`

**Interfaces:**
- Consumes: `botix_driver` launch argument `scan_topic`, default `/scan`.
- Produces: `/scan_raw` from the bridge and `/scan` from
  `scan_to_scan_filter_chain` in both integrated modes.

- [x] Add failing tests that require the driver default `/scan`, integrated `/scan_raw`, and filter include in both navigation launches.
- [x] Add `scan_topic` to driver bringup and remap bridge `scan` to it.
- [x] Pass `scan_topic=/scan_raw` from mapping/navigation and include `lidar_filter.launch.py`.
- [x] Add the `lidar_filter` dependency to `botix_navigation`.
- [x] Run focused tests and launch argument inspection.

### Task 4: Diagnostics and SLAM Stability

**Files:**
- Modify: `ros2_ws/src/botix_navigation/rviz/mapping.rviz`
- Modify: `ros2_ws/src/botix_navigation/rviz/navigation.rviz`
- Modify: `ros2_ws/src/botix_navigation/config/slam_toolbox.yaml`
- Modify: `ros2_ws/src/botix_description/urdf/botix.urdf`
- Extend: `ros2_ws/src/botix_navigation/test/test_configuration.py`

**Interfaces:**
- Produces: red `/scan_raw`, green `/scan`, and stricter SLAM match acceptance.

- [x] Add failing tests for `max_laser_range=6.0`, travel thresholds `0.08`, link response `0.25`, loop responses `0.55/0.65`, variance `1.0`, and chain size `15`.
- [x] Update SLAM parameters exactly as specified without changing `min_pass_through`.
- [x] Add raw and filtered LaserScan displays with best-effort QoS to both RViz profiles.
- [x] Replace the URDF placeholder lidar comment with evidence-backed X/Y and estimated-Z documentation, without changing the transform.
- [x] Run focused tests.

### Task 5: Documentation and Static Verification

**Files:**
- Create: `ros2_ws/src/lidar_filter/README.md`
- Modify: `README.md`

**Interfaces:**
- Produces: operator documentation for filter topics, bounds, tuning, and launch commands.

- [x] Document `/scan_raw`, `/scan`, self-box bounds, the no-speckle safety rule, and how to compare displays in RViz.
- [x] Run all pytest tests.
- [x] Run `colcon build --symlink-install`, `colcon test`, and `colcon test-result --verbose`.
- [x] Run `git diff --check` and review tracked files.

### Task 6: Live Acceptance and Publish

**Files:**
- Runtime artifacts only under `/tmp/botix-lidar-filter-acceptance-20260824/`.

**Interfaces:**
- Validates: raw/filtered rates, endpoint preservation, removed self-returns, TF, map publication, and RViz.

- [x] Stop the existing mapping process and launch the new mapping stack against `10.117.41.202` without RViz.
- [x] Record paired raw/filtered scans by timestamp and transform endpoints with the live `laser_frame -> base_footprint` TF.
- [x] Verify filtered self-box endpoints become NaN and every outside endpoint/range/intensity remains identical.
- [x] Verify `/scan_raw`, `/scan`, `/odom`, `/map`, TF, and publisher ownership.
- [x] Compare a saved stationary map and map-to-odom transform behavior with the captured baseline.
- [x] Run mapping with RViz and leave it open for operator inspection.
- [x] Commit all tracked changes, push `main`, and report limitations requiring a physical drive near chair/table legs.
