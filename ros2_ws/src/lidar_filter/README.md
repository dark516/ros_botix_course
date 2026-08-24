# lidar_filter

Geometric lidar self-filter for Botix. It wraps the standard ROS 2
`laser_filters/LaserScanBoxFilter` and removes endpoints inside the volume that
is physically occupied by the robot:

| Axis | Minimum | Maximum |
| :-- | --: | --: |
| X | -0.140 m | 0.140 m |
| Y | -0.115 m | 0.115 m |
| Z | 0.000 m | 0.200 m |

The filter transforms endpoints into `base_footprint`. It does not apply a
range threshold, angle mask, median filter, or speckle filter. Consequently,
every return outside the robot box, including an isolated chair leg, is passed
through unchanged.

Integrated Botix mapping and navigation use:

```text
/scan_raw -> scan_to_scan_filter_chain -> /scan -> SLAM, AMCL and Nav2 costmaps
```

Both Navigation2 costmaps use the same `0.28 x 0.23 m` X/Y envelope. This
prevents the filter from hiding space that Navigation2 would otherwise
consider outside the robot body.

Run the filter independently after arranging a raw scan publisher:

```bash
ros2 launch lidar_filter lidar_filter.launch.py \
  input_scan:=/scan_raw output_scan:=/scan
```

The mapping and navigation RViz profiles show raw points in translucent red
and filtered points in green. Tune the box only when a red point attached to
the robot remains visible in green. A real external obstacle must remain in
both displays.
