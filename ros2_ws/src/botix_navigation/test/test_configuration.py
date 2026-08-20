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
    expected = "[[0.10, 0.10], [0.10, -0.10], [-0.10, -0.10], [-0.10, 0.10]]"

    for name in ("local_costmap", "global_costmap"):
        costmap = params(nav2[name], name)
        assert costmap["footprint"] == expected
        assert costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
