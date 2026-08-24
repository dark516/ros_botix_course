"""Serialize the active SLAM Toolbox map and pose graph."""

import argparse
from pathlib import Path
import subprocess

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


def map_prefix(value: str) -> Path:
    path = Path(value).expanduser().resolve()
    if path.suffix:
        raise argparse.ArgumentTypeError("use a map prefix without an extension")
    if any(
        path.with_suffix(suffix).exists()
        for suffix in (".yaml", ".pgm", ".posegraph", ".data")
    ):
        raise argparse.ArgumentTypeError(f"map already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def save_occupancy_map(prefix: Path, run=subprocess.run) -> bool:
    command = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(prefix),
    ]
    try:
        result = run(command, check=False, timeout=30.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def cleanup_outputs(prefix: Path) -> None:
    for suffix in (".yaml", ".pgm", ".posegraph", ".data"):
        prefix.with_suffix(suffix).unlink(missing_ok=True)


class MapSerializer(Node):
    def __init__(self) -> None:
        super().__init__("botix_save_map")
        self.graph_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )

    def save(self, prefix: Path) -> bool:
        success = False
        try:
            if not save_occupancy_map(prefix):
                self.get_logger().error("Nav2 occupancy map save failed")
                return False

            if not self.graph_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("SLAM Toolbox serialize service is unavailable")
                return False
            graph_request = SerializePoseGraph.Request()
            graph_request.filename = str(prefix)
            future = self.graph_client.call_async(graph_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            graph_response = future.result()
            success = (
                graph_response is not None
                and graph_response.result == graph_response.RESULT_SUCCESS
            )
            return success
        finally:
            if not success:
                cleanup_outputs(prefix)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=map_prefix, help="output path without extension")
    parsed = parser.parse_args(args)
    rclpy.init()
    node = MapSerializer()
    try:
        if not node.save(parsed.prefix):
            raise SystemExit("failed to serialize map")
        node.get_logger().info(f"saved Nav2 map and pose graph to {parsed.prefix}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
