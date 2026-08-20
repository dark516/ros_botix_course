# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Bridges the Botix rover onto the ROS graph.

Two UDP sockets, because the robot keeps the streams apart:

  * MAVLink on ``local_port`` carries WHEEL_DISTANCE and the NAMED_VALUE_INT
    pair holding raw encoder counts, and takes MANUAL_CONTROL back.
  * Raw lidar bytes arrive on ``lidar_port``.

Published: /scan, /odom, /joint_states, /wheel_ticks, and odom TF.
Subscribed: /cmd_vel.
"""

from __future__ import annotations

import math
import socket

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster

from botix_driver.lidar import FrameParser, ScanAssembler
from botix_driver.odometry import (
    DifferentialDriveOdometry,
    planar_covariance,
    wheel_distances,
)

try:
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError as error:  # pragma: no cover
    raise SystemExit("pymavlink is required: pip install pymavlink") from error


class _UdpWriter:
    def __init__(self, sock: socket.socket, address: tuple[str, int]) -> None:
        self._sock = sock
        self._address = address

    def write(self, data: bytes) -> None:
        try:
            self._sock.sendto(data, self._address)
        except OSError:
            # A transient ARP or route failure must not take the node down
            pass


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class BotixBridge(Node):

    def __init__(self) -> None:
        super().__init__("botix_bridge")

        # connection
        self.declare_parameter("robot_host", "botix.local")
        self.declare_parameter("robot_port", 14550)
        self.declare_parameter("local_port", 14555)
        self.declare_parameter("lidar_port", 14560)
        self.declare_parameter("source_system", 255)
        self.declare_parameter("target_system", 1)

        # kinematics
        self.declare_parameter("wheel_separation", 0.175)
        self.declare_parameter("wheel_radius", 0.0335)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 3.0)

        # Encoder direction and distance scale are calibrated on the ESP32.
        # The tank mixer computes left = z + r, so a positive r turns the robot
        # clockwise, while positive angular.z in ROS is counter-clockwise.
        self.declare_parameter("invert_turn", True)

        # lidar
        self.declare_parameter("scan_bins", 360)
        self.declare_parameter("range_min", 0.02)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("lidar_frame", "laser_frame")
        self.declare_parameter("angle_offset_deg", 0.0)

        # frames
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("pose_xy_variance", 0.02)
        self.declare_parameter("pose_yaw_variance", 0.05)
        self.declare_parameter("twist_linear_variance", 0.04)
        self.declare_parameter("twist_angular_variance", 0.08)

        # control
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)

        self._read_parameters()

        self._open_sockets()

        self.scan_publisher = self.create_publisher(LaserScan, "scan", QoSPresetProfiles.SENSOR_DATA.value)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.joint_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tick_publisher = self.create_publisher(Int64MultiArray, "wheel_ticks", 10)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.parser = FrameParser()
        self.assembler = ScanAssembler(self.scan_bins, self.range_min, self.range_max)

        self._ticks: tuple[int | None, int | None] = (None, None)
        self.odometry = DifferentialDriveOdometry(
            self.wheel_separation, self.wheel_radius
        )
        self._last_odom_time = self.get_clock().now()

        self._drive = 0
        self._turn = 0
        self._last_command_time = self.get_clock().now()

        self.create_timer(0.005, self._poll_sockets)
        self.create_timer(1.0 / self.command_rate, self._send_command)

        self.get_logger().info(
            f"bridging {self.robot_host}:{self.robot_port} "
            f"(mavlink in {self.local_port}, lidar in {self.lidar_port})"
        )

    # setup

    def _read_parameters(self) -> None:
        get = self.get_parameter

        self.robot_host = get("robot_host").value
        self.robot_port = get("robot_port").value
        self.local_port = get("local_port").value
        self.lidar_port = get("lidar_port").value

        self.wheel_separation = get("wheel_separation").value
        self.wheel_radius = get("wheel_radius").value
        self.max_linear_speed = get("max_linear_speed").value
        self.max_angular_speed = get("max_angular_speed").value
        self.invert_turn = get("invert_turn").value

        self.scan_bins = get("scan_bins").value
        self.range_min = get("range_min").value
        self.range_max = get("range_max").value
        self.lidar_frame = get("lidar_frame").value
        self.angle_offset = math.radians(get("angle_offset_deg").value)

        self.odom_frame = get("odom_frame").value
        self.base_frame = get("base_frame").value
        self.publish_tf = get("publish_tf").value
        self.pose_covariance = planar_covariance(
            get("pose_xy_variance").value,
            get("pose_xy_variance").value,
            get("pose_yaw_variance").value,
        )
        self.twist_covariance = planar_covariance(
            get("twist_linear_variance").value,
            1e6,
            get("twist_angular_variance").value,
        )

        self.command_rate = get("command_rate").value
        self.command_timeout = get("command_timeout").value

    def _open_sockets(self) -> None:
        address = (socket.gethostbyname(self.robot_host), self.robot_port)

        self.mavlink_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mavlink_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mavlink_socket.bind(("0.0.0.0", self.local_port))
        self.mavlink_socket.setblocking(False)

        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidar_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.lidar_socket.bind(("0.0.0.0", self.lidar_port))
        self.lidar_socket.setblocking(False)

        self.mav = mavlink2.MAVLink(
            _UdpWriter(self.mavlink_socket, address),
            srcSystem=self.get_parameter("source_system").value,
            srcComponent=mavlink2.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.mav.robust_parsing = True
        self.target_system = self.get_parameter("target_system").value

    # inbound

    def _poll_sockets(self) -> None:
        while True:
            try:
                data, _ = self.mavlink_socket.recvfrom(2048)
            except (BlockingIOError, OSError):
                break

            for message in self.mav.parse_buffer(data) or []:
                self._on_mavlink(message)

        while True:
            try:
                datagram, _ = self.lidar_socket.recvfrom(2048)
            except (BlockingIOError, OSError):
                break

            for frame in self.parser.feed_datagram(datagram):
                completed = self.assembler.add(frame)
                if completed is not None:
                    self._publish_scan(*completed)

    def _on_mavlink(self, message) -> None:
        kind = message.get_type()

        if kind == "WHEEL_DISTANCE":
            distances = wheel_distances(message)
            if distances is not None:
                self._update_odometry(*distances)
            return

        if kind != "NAMED_VALUE_INT":
            return

        name = message.name.rstrip("\x00")

        left, right = self._ticks

        if name == "enc_left":
            left = message.value
        elif name == "enc_right":
            right = message.value
        else:
            return

        self._ticks = (left, right)

        # The firmware emits both halves back to back; integrate once paired
        if left is not None and right is not None:
            tick_message = Int64MultiArray()
            tick_message.data = [left, right]
            self.tick_publisher.publish(tick_message)
            self._ticks = (None, None)

    # odometry

    def _update_odometry(self, left: float, right: float) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now
        update = self.odometry.update(left, right, elapsed)
        if update is None:
            return

        self._publish_odometry(
            now, update.pose, update.linear_velocity, update.angular_velocity
        )
        self._publish_joints(
            now, update.wheel_positions, update.wheel_velocities
        )

    def _publish_odometry(self, stamp, pose, linear: float, angular: float) -> None:
        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame

        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation = yaw_to_quaternion(pose[2])
        message.pose.covariance = self.pose_covariance

        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        message.twist.covariance = self.twist_covariance

        self.odom_publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = pose[0]
            transform.transform.translation.y = pose[1]
            transform.transform.rotation = yaw_to_quaternion(pose[2])
            self.tf_broadcaster.sendTransform(transform)

    def _publish_joints(self, stamp, positions, velocities) -> None:
        message = JointState()
        message.header.stamp = stamp.to_msg()
        message.name = ["left_wheel_joint", "right_wheel_joint"]
        message.position = list(positions)
        message.velocity = list(velocities)
        self.joint_publisher.publish(message)

    # lidar

    def _publish_scan(self, ranges: list[float], intensities: list[float]) -> None:
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.lidar_frame

        message.angle_min = self.angle_offset
        message.angle_max = self.angle_offset + 2.0 * math.pi
        message.angle_increment = self.assembler.angle_increment
        message.range_min = self.range_min
        message.range_max = self.range_max
        message.ranges = ranges
        message.intensities = intensities

        self.scan_publisher.publish(message)

    # outbound

    def _on_cmd_vel(self, message: Twist) -> None:
        drive = message.linear.x / self.max_linear_speed
        turn = message.angular.z / self.max_angular_speed

        if self.invert_turn:
            turn = -turn

        self._drive = int(max(-1.0, min(1.0, drive)) * 1000)
        self._turn = int(max(-1.0, min(1.0, turn)) * 1000)
        self._last_command_time = self.get_clock().now()

    def _send_command(self) -> None:
        age = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9

        # Stop on a silent /cmd_vel rather than latching the last command
        if age > self.command_timeout:
            self._drive = 0
            self._turn = 0

        self.mav.manual_control_send(
            self.target_system, 0, 0, self._drive, self._turn, 0
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BotixBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.mav.manual_control_send(node.target_system, 0, 0, 0, 0, 0)
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
