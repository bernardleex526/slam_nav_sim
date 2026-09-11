from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sim_benchmark/scripts'))
from stage_b_runner import NavigationSession


class NavigationSessionTests(unittest.TestCase):
    def test_unknown_acceptance_prevents_next_goal(self):
        session = object.__new__(NavigationSession)
        session.node = Mock()
        session.node.get_clock.return_value.now.return_value = Time()
        session.client = Mock()
        session._wait = Mock(return_value=False)
        session.safe_to_continue = True
        status, _ = session.send_goal(1., 2., 0., 5.)
        self.assertEqual(status, 'TIMEOUT')
        self.assertFalse(session.safe_to_continue)


if __name__ == '__main__':
    unittest.main()
