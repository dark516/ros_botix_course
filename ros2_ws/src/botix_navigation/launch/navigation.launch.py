"""Launch Botix localization and autonomous Navigation2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_map(context):
    path = LaunchConfiguration("map").perform(context)
    if not path or not os.path.isfile(path):
        raise RuntimeError(f"map must name an existing YAML file: {path!r}")
    return []


def generate_launch_description():
    package = get_package_share_directory("botix_navigation")
    driver = get_package_share_directory("botix_driver")
    params = os.path.join(package, "config", "nav2.yaml")
    nav_nodes = [
        "controller_server", "smoother_server", "planner_server",
        "behavior_server", "bt_navigator", "waypoint_follower",
        "velocity_smoother",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("map", description="absolute path to a saved map YAML"),
        DeclareLaunchArgument("robot_host", default_value="botix.local"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_validate_map),
        GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(driver, "launch", "bringup.launch.py")),
                launch_arguments={"robot_host": LaunchConfiguration("robot_host"), "use_rviz": "false"}.items(),
            )],
        ),
        Node(package="botix_navigation", executable="cmd_mux", output="screen"),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[params, {"yaml_filename": LaunchConfiguration("map")}], output="screen",
        ),
        Node(package="nav2_amcl", executable="amcl", name="amcl", parameters=[params], output="screen"),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}],
            output="screen",
        ),
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", parameters=[params],
            remappings=[("cmd_vel", "/cmd_vel_nav_raw")], output="screen",
        ),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", parameters=[params], output="screen"),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", parameters=[params], output="screen"),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", parameters=[params],
            remappings=[("cmd_vel", "/cmd_vel_nav_raw")], output="screen",
        ),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", parameters=[params], output="screen"),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", parameters=[params], output="screen"),
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", parameters=[params],
            remappings=[("cmd_vel", "/cmd_vel_nav_raw"), ("cmd_vel_smoothed", "/cmd_vel_nav")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{"autostart": True, "node_names": nav_nodes}], output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(package, "rviz", "navigation.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")), output="screen",
        ),
    ])
