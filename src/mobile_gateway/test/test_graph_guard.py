"""ROS discovery guard contracts without a ROS or hardware dependency."""
import unittest
from types import SimpleNamespace

from mobile_gateway.node import check_external_stack
from mobile_gateway.operations import OperationError


class FakeGraph:
    def __init__(self, names=(), publishers=()):
        self.names = list(names)
        self.publishers = list(publishers)

    def get_node_names(self):
        return self.names

    def get_publishers_info_by_topic(self, topic):
        assert topic == '/cmd_vel'
        return [SimpleNamespace(node_name=name) for name in self.publishers]


class GraphGuardTests(unittest.TestCase):
    def assert_blocked(self, graph, owned, name):
        with self.assertRaises(OperationError) as caught:
            check_external_stack(graph, owned)
        self.assertEqual((caught.exception.status, caught.exception.code), (409, 'external_stack'))
        self.assertIn(name, str(caught.exception))

    def test_unowned_known_stack_node_is_rejected(self):
        self.assert_blocked(FakeGraph(names=['mobile_gateway', 'roboclaw_driver']), set(), 'roboclaw_driver')

    def test_unowned_cmd_vel_publisher_is_rejected_even_if_unknown(self):
        self.assert_blocked(FakeGraph(publishers=['external_teleop']), {'motors'}, 'external_teleop')

    def test_duplicate_known_node_is_rejected_even_if_owned(self):
        self.assert_blocked(FakeGraph(names=['roboclaw_driver', 'roboclaw_driver']), {'motors'}, 'roboclaw_driver')

    def test_owned_stack_and_unrelated_nodes_are_allowed(self):
        check_external_stack(FakeGraph(names=['mobile_gateway', 'roboclaw_driver', 'diagnostics'],
                                       publishers=['roboclaw_driver']), {'motors'})


if __name__ == '__main__':
    unittest.main()
