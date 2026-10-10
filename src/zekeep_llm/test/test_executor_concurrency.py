"""Exercise real ROS wait sets with fake services only, never motor commands."""
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from zekeep_llm.robot import RobotTools


class ExecutorConcurrencyTest(unittest.TestCase):
    def test_robot_polling_cancellation_and_lease_have_independent_wait_sets(self):
        rclpy.init()
        robot = RobotTools('executor_test')
        lease_node = Node('executor_test_lease')
        lease_executor = SingleThreadedExecutor(context=lease_node.context)
        lease_executor.add_node(lease_node)
        client = lease_node.create_client(Trigger, '/executor_test/lease')
        def respond(request, response):
            response.success = True
            return response
        lease_node.create_service(Trigger, '/executor_test/lease', respond)
        robot.create_timer(.001, lambda: None)
        stop = threading.Event()
        errors, replies = [], []
        threads = []
        def poll():
            try:
                while not stop.is_set():
                    robot.poll()
            except Exception as exc:
                errors.append(exc)
                stop.set()
        def renew():
            try:
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                for _ in range(30):
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(lease_node, future, executor=lease_executor, timeout_sec=1)
                    self.assertTrue(future.done())
                    self.assertTrue(future.result().success)
                    replies.append(True)
            except Exception as exc:
                errors.append(exc)
            finally:
                stop.set()
        try:
            with patch('rclpy.get_global_executor', side_effect=AssertionError('global executor must not be used')):
                threads = [threading.Thread(target=work, daemon=True) for work in (poll, poll, renew)]
                for thread in threads:
                    thread.start()
                threads[-1].join(timeout=8)
                stop.set()
                for thread in threads:
                    thread.join(timeout=2)
                self.assertFalse(any(thread.is_alive() for thread in threads))
                self.assertEqual(errors, [])
                self.assertEqual(len(replies), 30)
                self.assertNotEqual(robot.executor, lease_node.executor)

                # Cancellation still interrupts a pending motion wait.
                robot.cancel_event = threading.Event()
                robot._require_live_telemetry = lambda: None
                timer = robot.create_timer(.01, robot.cancel_event.set)
                with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                    robot._spin_until(lambda: False, 1, 'timeout', monitor_motion=True)
                robot.destroy_timer(timer)
        finally:
            stop.set()
            for thread in threads:
                thread.join(timeout=2)
            robot.destroy_node()
            lease_executor.shutdown()
            lease_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
