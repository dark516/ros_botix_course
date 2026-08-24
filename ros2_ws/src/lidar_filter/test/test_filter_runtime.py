import math
import os
import time
import unittest
from threading import Event, Thread

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import launch_ros.actions
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


INPUT_TOPIC = "/lidar_filter_test/scan_raw"
OUTPUT_TOPIC = "/lidar_filter_test/scan"


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.join(
        get_package_share_directory("lidar_filter"), "config", "self_filter.yaml"
    )
    filter_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config],
        remappings=[("scan", INPUT_TOPIC), ("scan_filtered", OUTPUT_TOPIC)],
    )
    static_transform = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0835",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "laser_frame",
        ],
    )
    return launch.LaunchDescription([
        static_transform,
        filter_node,
        launch_testing.actions.ReadyToTest(),
    ])


class FilterFixture(Node):
    def __init__(self):
        super().__init__("lidar_filter_runtime_test")
        self.received = {}
        self.message_event = Event()
        self.publisher = self.create_publisher(
            LaserScan, INPUT_TOPIC, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan, OUTPUT_TOPIC, self._receive, qos_profile_sensor_data
        )
        self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

    @staticmethod
    def stamp_key(message):
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def _receive(self, message):
        self.received[self.stamp_key(message)] = message
        self.message_event.set()

    def wait_for_filter(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() == 1:
                return True
            time.sleep(0.05)
        return False

    def make_scan(self, frame_id):
        scan = LaserScan()
        scan.header.frame_id = frame_id
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 2.0
        scan.range_min = 0.01
        scan.range_max = 10.0
        scan.ranges = [0.05, 0.20, 0.20, 0.05, 0.30]
        scan.intensities = [10.0, 20.0, 30.0, 40.0, 50.0]
        return scan

    def publish_until_received(self, scan, timeout=3.0):
        key = self.stamp_key(scan)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publisher.publish(scan)
            if self.message_event.wait(0.1) and key in self.received:
                return self.received[key]
            self.message_event.clear()
        return None


class TestFilterRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = FilterFixture()
        assert cls.node.wait_for_filter(), "filter did not subscribe to the raw scan"

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.node.spin_thread.join(timeout=2.0)

    def test_graph_has_unique_filter_nodes(self):
        deadline = time.monotonic() + 5.0
        node_names = []
        while time.monotonic() < deadline:
            node_names = self.node.get_node_names()
            if (
                "scan_to_scan_filter_chain" in node_names
                and "laser_scan_box_filter" in node_names
            ):
                break
            time.sleep(0.05)
        self.assertEqual(node_names.count("scan_to_scan_filter_chain"), 1)
        self.assertEqual(node_names.count("laser_scan_box_filter"), 1)

    def test_inside_points_are_nan_and_outside_data_is_unchanged(self):
        scan = self.node.make_scan("laser_frame")
        filtered = self.node.publish_until_received(scan)
        self.assertIsNotNone(filtered)

        self.assertTrue(math.isnan(filtered.ranges[0]))
        self.assertEqual(filtered.ranges[1], scan.ranges[1])
        self.assertEqual(filtered.ranges[2], scan.ranges[2])
        self.assertTrue(math.isnan(filtered.ranges[3]))
        self.assertEqual(filtered.ranges[4], scan.ranges[4])
        self.assertEqual(filtered.intensities, scan.intensities)

    def test_missing_transform_does_not_publish_a_scan(self):
        scan = self.node.make_scan("missing_laser_frame")
        key = self.node.stamp_key(scan)
        for _ in range(5):
            self.node.publisher.publish(scan)
            time.sleep(0.1)
        time.sleep(0.5)
        self.assertNotIn(key, self.node.received)
