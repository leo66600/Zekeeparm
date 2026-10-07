"""Run with PYTHONPATH=src python3 -m servo_gui.test_calibrate_zero; no hardware."""

import unittest
from unittest.mock import Mock, call, patch

from .bus_worker import BusWorker


class CalibrateZeroTest(unittest.TestCase):
    def test_release_sequence_and_guards(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            with patch.object(bus, "command", wraps=bus.command) as commands:
                result = worker.calibrate_zero(1, "CALIBRATE 001 TO 1500")
                self.assertTrue(result["ok"], result)
                self.assertEqual(result["after"], 1500)
                self.assertEqual(commands.call_args_list, [call("#001PULK!"), call("#001PSCK!"),
                                                          call("#001PULK!"), call("#001PULK!")])
            for responses in [("",), ("#OK!", "#OK!", "")]:
                with patch.object(worker, "_command_wait", side_effect=responses) as send:
                    result = worker.calibrate_zero(4, "CALIBRATE 004 TO 1500")
                    self.assertEqual(result["error"]["code"], "torque_release_failed")
                    if len(responses) == 1:
                        send.assert_called_once_with("#004PULK!")
                    else:
                        self.assertIsNone(result["torque_restored"])
            with patch.object(worker, "_command_wait") as send:
                self.assertEqual(worker.calibrate_zero(2, "CALIBRATE 002 TO 2400")["error"]["code"], "manual_only")
                self.assertEqual(worker.calibrate_zero(1, "wrong")["error"]["code"], "confirm_mismatch")
                send.assert_not_called()
            with patch.object(bus, "read_pwm", side_effect=[1460, 1460, 1470, 1460]), \
                    patch.object(worker, "_command_wait", return_value="#OK!") as send:
                self.assertEqual(worker.calibrate_zero(1, "CALIBRATE 001 TO 1500")["error"]["code"], "joint_not_stable")
                send.assert_called_once_with("#001PULK!")
        finally:
            worker.shutdown()

    def test_release_repeated_through_save_without_repeating_calibration(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            for terminator in ["", "\r\n"]:
                serial_port = Mock()
                serial_port.read_all.return_value = b""
                with patch.object(bus, "serial", serial_port, create=True), \
                        patch.object(bus, "terminator", terminator, create=True), \
                        patch.object(worker, "_simulate", False), \
                        patch("servo_gui.bus_worker.time.monotonic", side_effect=[0, 0, 0.01, 0.02, 0.36]), \
                        patch("servo_gui.bus_worker.time.sleep") as sleep:
                    self.assertEqual(worker._command_wait("#001PSCK!", followup="#001PULK!"), "")
                    self.assertEqual(serial_port.write.call_args_list,
                                     [call(f"#001PSCK!{terminator}".encode("ascii"))]
                                     + [call(f"#001PULK!{terminator}".encode("ascii"))] * 3)
                    self.assertEqual(sleep.call_args_list, [call(0.01)] * 3)
                    serial_port.read_all.assert_called_once_with()
        finally:
            worker.shutdown()

    def test_manual_confirmation_for_silent_release(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            send = bus.command
            def silent_release(message):
                return "" if message == "#007PULK!" else send(message)
            with patch.object(bus, "command", side_effect=silent_release) as commands:
                self.assertEqual(worker.calibrate_zero(7, "CALIBRATE 007 TO 1500")["error"]["code"], "torque_release_failed")
                self.assertNotIn(call("#007PSCK!"), commands.call_args_list)
                result = worker.calibrate_zero(7, "CALIBRATE 007 TO 1500", manual_release_confirmed=True)
                self.assertTrue(result["ok"], result)
                self.assertEqual(result["after"], 1500)
                self.assertIsNone(result["torque_restored"])
                self.assertFalse(result["torque_release_acknowledged"])
                self.assertTrue(result["manual_release_confirmed"])
                # The manual confirmation does not persist into another request.
                self.assertEqual(worker.calibrate_zero(7, "CALIBRATE 007 TO 1500")["error"]["code"], "torque_release_failed")
            for response in ["#ERROR!", "#007PULK!"]:
                with patch.object(worker, "_command_wait", return_value=response) as commands:
                    self.assertEqual(worker.calibrate_zero(7, "CALIBRATE 007 TO 1500", manual_release_confirmed=True)["error"]["code"], "torque_release_failed")
                    commands.assert_called_once_with("#007PULK!")
            for value in [1, "true", None]:
                self.assertEqual(worker.calibrate_zero(7, "CALIBRATE 007 TO 1500", manual_release_confirmed=value)["error"]["code"], "invalid_request")
        finally:
            worker.shutdown()


if __name__ == "__main__":
    unittest.main()
