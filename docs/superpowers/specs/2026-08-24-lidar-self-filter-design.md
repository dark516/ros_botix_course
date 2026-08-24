# Botix Lidar Self-Filter Design

## Goal

Remove laser returns caused by the Botix chassis, electronics, wheels, and
wiring without hiding any obstacle outside the robot. Feed the same filtered
scan to SLAM Toolbox, AMCL, and both Navigation2 obstacle layers, while keeping
the raw scan available for diagnostics.

## Evidence

The captured map was 225 by 252 cells at 0.05 m resolution. Its occupied cells
formed 284 disconnected components, including 127 one-cell and 41 two-cell
components. A 45-scan sample contained only about 99 finite bins per 360-bin
scan and found persistent returns between approximately 139 and 197 degrees at
47 to 162 mm. Those returns move with the robot and create false occupied cells
throughout the map.

The top-view photograph places the lidar center approximately 55 to 60 mm in
front of the wheel axis and on the longitudinal centerline. The existing URDF
transform (`x=0.055 m`, `y=0`) is therefore consistent with available evidence.
The image does not provide a trustworthy scan-plane height measurement, and
height does not affect planar scan geometry, so the existing Z transform stays
unchanged until it can be measured directly.

## Package Boundary

Create an ament package named `lidar_filter`. It owns:

- the `laser_filters` chain configuration;
- a reusable `lidar_filter.launch.py` launch file;
- package-level configuration and launch contract tests;
- documentation for raw/filtered topics and geometric tuning.

The package does not parse lidar frames, estimate poses, modify odometry, or
implement a custom filtering algorithm. It wraps the standard Jazzy
`laser_filters/LaserScanBoxFilter` plugin.

## Topic Flow

In integrated mapping and navigation modes:

```text
botix_driver /scan_raw
        |
        v
lidar_filter LaserScanBoxFilter
        |
        +-- /scan --> SLAM Toolbox or AMCL
                  --> Nav2 local obstacle layer
                  --> Nav2 global obstacle layer
                  --> filtered RViz display

/scan_raw ----------------------> raw RViz diagnostic display
```

`botix_driver` keeps `/scan` as its default topic when launched alone, so its
existing public interface remains compatible. Its bringup launch gains a
`scan_topic` argument. Mapping and navigation pass `/scan_raw`, then start
`lidar_filter`, which publishes the established `/scan` topic.

## Filter Geometry

The filter transforms every endpoint from `laser_frame` into
`base_footprint`, then removes endpoints strictly inside this box:

```text
x: -0.140 m .. +0.140 m
y: -0.115 m .. +0.115 m
z:  0.000 m .. +0.200 m
```

This box encloses the chassis, wheels, battery holder, electronics, and loose
wiring visible in the photographs. Space inside it is already physically
occupied by the robot, so removing those endpoints cannot hide a reachable
external obstacle.

The filter must preserve every range and intensity outside the box unchanged.
No range floor, angular mask, median filter, temporal filter, interpolation, or
speckle filter is permitted. In particular, isolated returns from chair and
table legs remain visible to Navigation2.

Invalid input values remain invalid. When the required TF is unavailable, the
filter reports the failure and does not publish a misleading partially
filtered scan.

## URDF

Keep the existing laser joint translation because the top-view image supports
its X/Y values and the available photos cannot measure Z reliably. Replace the
placeholder comment with the measured/observed basis for X/Y and document that
Z is an estimate pending direct measurement. No speculative orientation or
height adjustment is part of this change.

## Mapping Stability

Filtering self-returns and tuning SLAM are separate changes and are validated
separately. After the self-filter is active:

- reduce `max_laser_range` from 8 m to 6 m;
- increase `minimum_travel_distance` and `minimum_travel_heading` from 0.05 to
  0.08 so nearly identical sparse scans are not inserted excessively;
- increase `link_match_minimum_response_fine` from 0.10 to 0.25;
- increase loop-closure response thresholds from 0.35/0.45 to 0.55/0.65;
- reduce `loop_match_maximum_variance_coarse` from 3.0 to 1.0;
- increase `loop_match_minimum_chain_size` from 10 to 15.

These values reject weak sequential matches and weak loop closures instead of
removing external measurements. Loop closure remains enabled, but a candidate
must have stronger evidence before it can change `map -> odom`.

`min_pass_through` is not changed in the first implementation. Any later map
occupancy tuning must remain internal to SLAM; it must never alter the scan
used by Navigation2 obstacle layers.

## RViz

Mapping and navigation RViz profiles show:

- `/scan_raw` in red with low opacity;
- `/scan` in green at normal opacity;
- the robot model, odometry, map, and existing navigation displays.

This makes incorrect self-box dimensions immediately visible. A valid external
return must appear at the same location in both scan displays.

## Testing

Automated tests verify:

- package metadata and installation of launch/config assets;
- the filter type is exactly `LaserScanBoxFilter`;
- box frame, bounds, and `invert=false` match this specification;
- no second filter exists in the chain;
- driver standalone defaults to `/scan`;
- mapping and navigation route the driver to `/scan_raw` and start the filter;
- SLAM, AMCL, and costmaps continue consuming `/scan`;
- RViz contains both raw and filtered displays;
- the strengthened SLAM thresholds match this specification.

Live acceptance records raw and filtered scans concurrently and verifies:

- `/scan_raw` and `/scan` publish at the lidar rate;
- persistent 47 to 162 mm body returns disappear from `/scan`;
- all finite raw endpoints outside the self-box have identical filtered ranges
  and intensities;
- `/map`, `/odom`, TF, and Nav2 consumers remain connected;
- no second publisher appears on either scan topic.

A new short mapping run is compared with the captured baseline for isolated
occupied components and unexpected `map -> odom` jumps. Physical navigation
acceptance still requires driving near real chair/table legs; those returns are
checked in raw and filtered RViz before autonomous goals are allowed.

## Operations

`mapping.launch.py` and `navigation.launch.py` start the filter automatically.
Operators normally consume `/scan`; `/scan_raw` is diagnostic. The standalone
filter launch supports explicit input/output remappings for troubleshooting.

Generated bags and maps are runtime artifacts and are not committed.
