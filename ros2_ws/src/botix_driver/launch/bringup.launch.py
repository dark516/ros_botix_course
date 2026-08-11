# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Bring up the bridge, optionally with a static transform for the lidar."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("botix_driver")
    default_parameters = os.path.join(package_share, "config", "botix.yaml")

    parameters_argument = DeclareLaunchArgument(
        "params", default_value=default_parameters, description="parameter file"
    )
    host_argument = DeclareLaunchArgument(
        "robot_host", default_value="botix.local", description="robot hostname or IP"
    )

    bridge = Node(
        package="botix_driver",
        executable="bridge",
        name="botix_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params"),
            {"robot_host": LaunchConfiguration("robot_host")},
        ],
    )

    # Where the lidar sits relative to the chassis. Measure and correct these
    # before mapping: an offset here shows up as walls that drift when turning.
    laser_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["0", "0", "0.08", "0", "0", "0", "base_link", "laser"],
    )

    return LaunchDescription([parameters_argument, host_argument, bridge, laser_transform])
