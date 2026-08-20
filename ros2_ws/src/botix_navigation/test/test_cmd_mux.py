from geometry_msgs.msg import Twist

from botix_navigation.cmd_mux import CommandState, select_command


def command(speed):
    message = Twist()
    message.linear.x = speed
    return message


def test_teleop_overrides_navigation():
    selected = select_command(
        [CommandState(command(0.1), 1.0, 50), CommandState(command(0.2), 1.1, 100)],
        now=1.2,
        timeout=0.5,
        locked=False,
    )
    assert selected.linear.x == 0.2


def test_expired_commands_and_lock_return_no_command():
    states = [CommandState(command(0.1), 1.0, 50)]
    assert select_command(states, 2.0, 0.5, False) is None
    assert select_command(states, 1.1, 0.5, True) is None
