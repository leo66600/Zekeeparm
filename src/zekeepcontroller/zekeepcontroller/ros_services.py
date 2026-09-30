from __future__ import annotations

from collections.abc import Callable
from typing import Any, TypeVar

from rclpy.node import Node
from zekeep_msgs.srv import (
    GripperCommand,
    MoveToPoseIK,
    SetGripper,
    SetZero,
)
from std_srvs.srv import Trigger

from .conversions import pose_to_xyz_rpy

ResponseT = TypeVar("ResponseT")


class ArmServices:
    """Register ROS services that operate the arm and gripper safely."""

    def __init__(self, node: Node, hardware: Any, namespace: str) -> None:
        self._node = node
        self._hardware = hardware

        services = (
            (Trigger, "enable", self.enable, node.slow_group),
            (Trigger, "disable", self.disable, node.reentrant_group),
            (Trigger, "stop", self.stop, node.reentrant_group),
            (Trigger, "safe_home", self.safe_home, node.slow_group),
            (Trigger, "return_ready", self.return_ready, node.slow_group),
            (
                Trigger,
                "gravity_compensation/start",
                self.start_gravity_compensation,
                node.slow_group,
            ),
            (
                Trigger,
                "gravity_compensation/stop",
                self.stop_gravity_compensation,
                node.slow_group,
            ),
            (
                Trigger,
                "gravity_compensation/status",
                self.gravity_compensation_status,
                node.reentrant_group,
            ),
            (SetZero, "set_zero", self.set_zero, node.slow_group),
            (MoveToPoseIK, "move_to_pose_ik", self.move_to_pose_ik, node.reentrant_group),
            (SetGripper, "gripper/set", self.set_gripper, node.reentrant_group),
            (GripperCommand, "gripper/open", self.open_gripper, node.slow_group),
            (GripperCommand, "gripper/close", self.close_gripper, node.slow_group),
        )
        for srv_type, name, handler, group in services:
            node.create_service(
                srv_type,
                f"/{namespace}/{name}",
                handler,
                callback_group=group,
            )

    def _run(
        self,
        response: ResponseT,
        action: Callable[[], None],
        success_message: str,
        *,
        read_hardware: bool = True,
    ) -> ResponseT:
        try:
            action()
            response.success = True
            response.message = success_message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status(read_hardware=read_hardware)
        return response

    def enable(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run(response, self._hardware.enable, "enabled")

    def disable(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        def action() -> None:
            self._hardware.stop_gravity_compensation()
            self._hardware.disable()

        return self._run(response, action, "disabled", read_hardware=False)

    def stop(self, _request, response):
        return self._run(response, self._hardware.stop_and_hold, "stopped; holding position")

    def safe_home(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run(response, self._hardware.safe_home, "safe_home complete")

    def return_ready(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run(response, self._hardware.return_ready, "return_ready complete")

    def start_gravity_compensation(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run(
            response,
            self._hardware.start_gravity_compensation,
            "gravity compensation started",
        )

    def stop_gravity_compensation(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run(
            response,
            self._hardware.stop_gravity_compensation,
            "gravity compensation stopped",
        )

    def gravity_compensation_status(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Report state without changing motor mode or targets."""
        try:
            active = bool(
                self._hardware.gravity_compensation_active()
                or self._hardware.state_machine == "GRAVITY_COMP"
            )
            response.success = active
            response.message = (
                "gravity compensation running"
                if active
                else f"gravity compensation stopped (state={self._hardware.state_machine})"
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def set_zero(
        self, request: SetZero.Request, response: SetZero.Response
    ) -> SetZero.Response:
        def action() -> None:
            self._hardware.stop_gravity_compensation()
            if not self._hardware.set_zero(request.joint_name):
                raise RuntimeError("set_zero failed")

        return self._run(response, action, "set_zero complete")

    def move_to_pose_ik(
        self, request: MoveToPoseIK.Request, response: MoveToPoseIK.Response
    ) -> MoveToPoseIK.Response:
        try:
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(request.target_pose)
            ok, q_solution = self._hardware.solve_pose_ik(x, y, z, roll, pitch, yaw)
            response.success = ok
            response.message = "IK solution ready" if ok else "IK failed"
            response.q_solution = q_solution
        except Exception as exc:
            self._hardware.hold_current_position()
            response.success = False
            response.message = str(exc)
            response.q_solution = []
        self._node.publish_arm_status()
        return response

    def set_gripper(
        self, request: SetGripper.Request, response: SetGripper.Response
    ) -> SetGripper.Response:
        try:
            reached, reached_position = self._hardware.set_gripper_position(
                request.position,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
        except Exception as exc:
            response.success = False
            response.reached_position = 0.0
            self._node.get_logger().error(f"gripper set failed: {exc}")
        self._node.publish_arm_status()
        return response

    def open_gripper(
        self, request: GripperCommand.Request, response: GripperCommand.Response
    ) -> GripperCommand.Response:
        return self._move_gripper(
            request, response, self._hardware.gripper_open_position, "open"
        )

    def close_gripper(
        self, request: GripperCommand.Request, response: GripperCommand.Response
    ) -> GripperCommand.Response:
        return self._move_gripper(
            request, response, self._hardware.gripper_close_position, "close"
        )

    def _move_gripper(
        self,
        request: GripperCommand.Request,
        response: GripperCommand.Response,
        default_target: float,
        label: str,
    ) -> GripperCommand.Response:
        try:
            target = (
                default_target if request.position == 0.0 else float(request.position)
            )
            success, position = self._hardware.set_gripper_position(
                target,
                timeout=request.timeout if request.timeout > 0.0 else 3.0,
            )
            response.success = bool(success)
            response.reached_position = float(position)
            response.message = (
                f"gripper {label} complete" if success else f"gripper {label} timeout"
            )
        except Exception as exc:
            response.success = False
            response.reached_position = 0.0
            response.message = str(exc)
        self._node.publish_arm_status()
        return response
