"""Priority command arbiter with an emergency lock."""

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass(frozen=True)
class CommandState:
    command: Twist
    received_at: float
    priority: int


def select_command(states, now: float, timeout: float, locked: bool):
    if locked:
        return None
    active = [state for state in states if now - state.received_at <= timeout]
    return max(active, key=lambda state: state.priority).command if active else None


class CommandMux(Node):
    def __init__(self):
        super().__init__("botix_cmd_mux")
        self.declare_parameter("timeout", 0.5)
        self.timeout = float(self.get_parameter("timeout").value)
        self.states = {}
        self.locked = False
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Twist, "/cmd_vel_teleop", lambda msg: self._receive("teleop", msg, 100), 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_nav", lambda msg: self._receive("navigation", msg, 50), 10
        )
        self.create_subscription(Bool, "/cmd_vel_lock", self._lock, 10)
        self.create_timer(0.05, self._publish)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _receive(self, source, command, priority):
        self.states[source] = CommandState(command, self._now(), priority)

    def _lock(self, message):
        self.locked = message.data
        self._publish()

    def _publish(self):
        command = select_command(
            self.states.values(), self._now(), self.timeout, self.locked
        )
        self.publisher.publish(command if command is not None else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
