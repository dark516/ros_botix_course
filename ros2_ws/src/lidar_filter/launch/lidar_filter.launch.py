"""Run the geometric Botix lidar self-filter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("lidar_filter")
    config = os.path.join(package, "config", "self_filter.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("input_scan", default_value="/scan_raw"),
        DeclareLaunchArgument("output_scan", default_value="/scan"),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[config],
            remappings=[
                ("scan", LaunchConfiguration("input_scan")),
                ("scan_filtered", LaunchConfiguration("output_scan")),
            ],
            output="screen",
        ),
    ])
