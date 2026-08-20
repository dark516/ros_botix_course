"""Launch Botix hardware, command arbitration, and online mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("botix_navigation")
    driver = get_package_share_directory("botix_driver")
    slam = get_package_share_directory("slam_toolbox")

    return LaunchDescription([
        DeclareLaunchArgument("robot_host", default_value="botix.local"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver, "launch", "bringup.launch.py")),
            launch_arguments={
                "robot_host": LaunchConfiguration("robot_host"),
                "use_rviz": "false",
            }.items(),
        ),
        Node(package="botix_navigation", executable="cmd_mux", output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": os.path.join(package, "config", "slam_toolbox.yaml"),
                "use_sim_time": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(package, "rviz", "mapping.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ])
