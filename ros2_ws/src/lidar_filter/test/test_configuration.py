from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
SOURCE = PACKAGE.parent


def load_yaml(relative_path):
    with (PACKAGE / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_self_filter_removes_only_the_robot_box():
    document = load_yaml("config/self_filter.yaml")
    assert set(document) == {"scan_to_scan_filter_chain"}
    parameters = document["scan_to_scan_filter_chain"]["ros__parameters"]

    assert set(parameters) == {"filter1"}
    assert parameters["filter1"] == {
        "name": "self_box",
        "type": "laser_filters/LaserScanBoxFilter",
        "params": {
            "box_frame": "base_footprint",
            "min_x": -0.140,
            "max_x": 0.140,
            "min_y": -0.115,
            "max_y": 0.115,
            "min_z": 0.000,
            "max_z": 0.200,
            "invert": False,
        },
    }


def test_filter_launch_has_remappable_input_and_output():
    source = (PACKAGE / "launch/lidar_filter.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("input_scan", default_value="/scan_raw")' in source
    assert 'DeclareLaunchArgument("output_scan", default_value="/scan")' in source
    assert 'executable="scan_to_scan_filter_chain"' in source
    assert 'name="lidar_self_filter"' not in source
    assert '("scan", LaunchConfiguration("input_scan"))' in source
    assert '("scan_filtered", LaunchConfiguration("output_scan"))' in source

    manifest = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>ament_index_python</exec_depend>" in manifest


def test_driver_and_navigation_route_scans_through_filter():
    driver = (SOURCE / "botix_driver/launch/bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan")' in driver
    assert '("scan", LaunchConfiguration("scan_topic"))' in driver

    for relative in (
        "botix_navigation/launch/mapping.launch.py",
        "botix_navigation/launch/navigation.launch.py",
    ):
        source = (SOURCE / relative).read_text(encoding="utf-8")
        assert '"scan_topic": "/scan_raw"' in source
        assert 'get_package_share_directory("lidar_filter")' in source
        assert '"lidar_filter.launch.py"' in source
