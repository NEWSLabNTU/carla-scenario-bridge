import rclpy.logging

from swb_agent import protocol as P
from scenario_agent_relay.relay_node import RelayNode


class _Server:
    def __init__(self, replies):
        self.replies = list(replies)

    def command(self, entity, name, timeout, **args):
        return self.replies.pop(0)


class _Node:
    """Just what RelayNode._command touches."""

    entity = "ego"

    def __init__(self, replies):
        self.server = _Server(replies)
        self._logger = rclpy.logging.get_logger("test_relay_logging")

    def get_logger(self):
        return self._logger


def test_command_logs_ok_then_failure_without_raising():
    # rclpy raises "Logger severity cannot be changed between calls" when one source line
    # logs at two severities; a non-OK reply after an OK one used to kill the relay.
    node = _Node([(P.OK, ""), ("ERROR", "not ready"), (P.OK, "")])
    for _ in range(3):
        RelayNode._command(node, "set_speed_limit", 1.0)
