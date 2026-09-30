"""Shared launch helpers for the MoveIt bringup entry points."""

import os
from typing import Any

from launch_ros.actions import Node


def create_worktable_collision_node(
    robot_description_semantic: Any,
    wait_for_rviz: Any,
    *,
    condition: Any = None,
) -> Node:
    """Create the node that publishes the fixed worktable scene."""
    return Node(
        package="zekeep_moveit_config",
        executable="worktable_collision",
        output="screen",
        parameters=[
            robot_description_semantic,
            {"wait_for_rviz": wait_for_rviz},
        ],
        condition=condition,
    )


def build_moveit_parameters(moveit_config: Any) -> dict[str, Any]:
    """Build MoveIt parameters with project-specific planning defaults."""
    parameters = moveit_config.to_dict()
    robot_description_planning = parameters.setdefault(
        "robot_description_planning", {}
    )
    robot_description_planning["default_robot_padding"] = 0.02
    ompl = parameters.setdefault("ompl", {})
    ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"

    if os.environ.get("ROS_DISTRO") == "humble":
        ompl["request_adapters"] = " ".join(
            [
                "default_planner_request_adapters/"
                "AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
            ]
        )
        ompl.pop("response_adapters", None)

    return parameters
