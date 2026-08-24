# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Bring up the physical Botix robot and its visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("botix_driver")
    description_share = get_package_share_directory("botix_description")
    default_parameters = os.path.join(package_share, "config", "botix.yaml")
    robot_description_path = os.path.join(description_share, "urdf", "botix.urdf")
    rviz_config = os.path.join(package_share, "rviz", "botix.rviz")

    with open(robot_description_path, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    parameters_argument = DeclareLaunchArgument(
        "params", default_value=default_parameters, description="parameter file"
    )
    host_argument = DeclareLaunchArgument(
        "robot_host", default_value="botix.local", description="robot hostname or IP"
    )
    rviz_argument = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="start RViz2"
    )
    scan_argument = DeclareLaunchArgument("scan_topic", default_value="/scan")

    bridge = Node(
        package="botix_driver",
        executable="bridge",
        name="botix_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params"),
            {"robot_host": LaunchConfiguration("robot_host")},
        ],
        remappings=[("scan", LaunchConfiguration("scan_topic"))],
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            parameters_argument,
            host_argument,
            rviz_argument,
            scan_argument,
            state_publisher,
            bridge,
            rviz,
        ]
    )
