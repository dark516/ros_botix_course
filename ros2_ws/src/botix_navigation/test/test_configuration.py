from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / "config"


def load(name):
    with (CONFIG / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def params(document, node):
    return document[node]["ros__parameters"]


def test_twist_mux_priorities_and_timeouts():
    mux = params(load("twist_mux.yaml"), "twist_mux")

    assert mux["topics"]["teleop"] == {
        "topic": "/cmd_vel_teleop",
        "timeout": 0.5,
        "priority": 100,
    }
    assert mux["topics"]["navigation"] == {
        "topic": "/cmd_vel_nav",
        "timeout": 0.5,
        "priority": 50,
    }
    assert mux["locks"]["emergency"]["topic"] == "/cmd_vel_lock"
    assert mux["locks"]["emergency"]["priority"] == 255


def test_slam_frames_and_resolution():
    slam = params(load("slam_toolbox.yaml"), "slam_toolbox")

    assert slam["map_frame"] == "map"
    assert slam["odom_frame"] == "odom"
    assert slam["base_frame"] == "base_footprint"
    assert slam["scan_topic"] == "/scan"
    assert slam["resolution"] == 0.05
    assert slam["mode"] == "mapping"


def test_slam_constrains_scan_matching_to_odometry():
    slam = params(load("slam_toolbox.yaml"), "slam_toolbox")

    assert slam["max_laser_range"] == 6.0
    assert slam["minimum_travel_distance"] == 0.08
    assert slam["minimum_travel_heading"] == 0.08
    assert slam["link_match_minimum_response_fine"] == 0.5
    assert slam["do_loop_closing"] is False
    assert slam["correlation_search_space_dimension"] == 0.16
    assert slam["correlation_search_space_resolution"] == 0.01
    assert slam["correlation_search_space_smear_deviation"] == 0.03
    assert slam["distance_variance_penalty"] == 0.3
    assert slam["angle_variance_penalty"] == 0.34906585
    assert slam["coarse_search_angle_offset"] == 0.08726646
    assert slam["coarse_angle_resolution"] == 0.01745329
    assert slam["use_response_expansion"] is False
    assert "min_pass_through" not in slam


def test_rviz_compares_raw_and_filtered_scans():
    for name in ("mapping.rviz", "navigation.rviz"):
        source = (PACKAGE / "rviz" / name).read_text(encoding="utf-8")
        assert "Name: Raw Lidar" in source
        assert "Value: /scan_raw" in source
        assert "Name: Filtered Lidar" in source
        assert "Value: /scan" in source


def test_nav2_has_required_servers_and_safe_limits():
    nav2 = load("nav2.yaml")
    required = {
        "amcl",
        "map_server",
        "bt_navigator",
        "controller_server",
        "planner_server",
        "behavior_server",
        "local_costmap",
        "global_costmap",
        "velocity_smoother",
    }

    assert required <= nav2.keys()
    controller = params(nav2, "controller_server")
    assert controller["FollowPath"]["desired_linear_vel"] <= 0.18
    smoother = params(nav2, "velocity_smoother")
    assert smoother["max_velocity"] == [0.18, 0.0, 0.8]


def test_both_costmaps_use_the_measured_footprint_and_scan():
    nav2 = load("nav2.yaml")
    expected = (
        "[[0.14, 0.115], [0.14, -0.115], "
        "[-0.14, -0.115], [-0.14, 0.115]]"
    )

    for name in ("local_costmap", "global_costmap"):
        costmap = params(nav2[name], name)
        assert costmap["footprint"] == expected
        assert costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
