# botix_navigation

ROS 2 Jazzy bringup for mapping with SLAM Toolbox and map-based navigation with
AMCL and Navigation2. Mapping and navigation are separate launch modes so only
one node owns the `map -> odom` transform.

```bash
ros2 launch botix_navigation mapping.launch.py robot_host:=botix.local
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel_teleop
ros2 run botix_navigation save_map "$HOME/maps/botix_lab"
```

Stop mapping before starting navigation:

```bash
ros2 launch botix_navigation navigation.launch.py \
  map:="$HOME/maps/botix_lab.yaml" robot_host:=botix.local
```

The `botix_navigation` command mux is the sole physical `/cmd_vel` publisher in
both modes. Teleop overrides Nav2, and a true value on `/cmd_vel_lock` blocks
both sources.

The mapping profile constrains scan matching to a small neighborhood around
wheel odometry and disables automatic loop closure. This avoids large false
corrections from sparse low-cost lidar scans. Drive slowly, avoid wheel slip,
and use short, smooth turns before saving the map.
