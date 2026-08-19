# botix_description

ROS 2 Jazzy description package for the `botix` differential-drive robot.

This first version is intentionally simple:

- the kinematic structure is ready for `robot_state_publisher`
- the drive wheels are modeled with the CAD-verified 67.6 mm outer diameter
- the body dimensions follow the main FreeCAD assembly placements
- the visuals are primitive placeholders until the CAD parts are exported to ROS-friendly meshes
- `laser_frame` models the currently installed front-mounted Camsense lidar

## Launch

```bash
ros2 launch botix_description view_robot.launch.py
```

## Current assumptions

- `base_link` is centered between the drive wheel axles
- `base_footprint` is the floor-projected frame for navigation
- wheel separation is set to about `0.1754 m`
- wheel outer diameter is set to `0.0676 m`
- the lidar pose is an initial measurement and must be refined before mapping

## Next step

Calibrate wheel separation and the lidar mount pose before starting SLAM, then
replace the primitive visuals with exported meshes when they are available.
