#!/usr/bin/env python3
"""Keep the confirmed fixed worktable in the MoveIt planning scene."""

import math
from collections.abc import Iterable
from xml.etree import ElementTree
from typing import Any

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    LinkPadding,
    ObjectColor,
    PlanningScene,
)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


# August 15 CAD export base mesh bottom in base_link coordinates.  The tiny
# negative value is the binary STL's floating-point representation of zero.
BASE_BOTTOM_Z_M = -2.1640675362810669e-16
GRIPPER_COLLISION_PADDING_M = 0.003
GRIPPER_COLLISION_LINKS = ("gripper_base", "left_link", "right_link")


def worktable_publish_ready(
    planning_scene_subscribers: int,
    wait_for_rviz: bool,
    monitored_scene_subscribers: int,
) -> bool:
    """Return whether both MoveIt and the requested RViz consumer are ready."""
    return planning_scene_subscribers > 0 and (
        not wait_for_rviz or monitored_scene_subscribers > 0
    )


def create_worktable_geometry(
    length: float = 1.2,
    width: float = 0.9,
    height: float = 0.8,
    mount_inset: float = 0.062,
    base_bottom_z: float = BASE_BOTTOM_Z_M,
    top_clearance: float = 0.0,
) -> dict[str, Any]:
    """Return dimensions and pose in the fixed base_link frame, in metres."""
    inboard_distance = width / 2.0 - mount_inset
    return {
        # X is table width into the workspace; Y follows the 1200 mm edge.
        "dimensions": (width, length, height),
        # J1 is 62 mm inboard from the near edge. The effective installed
        # tabletop plane follows the active CAD base mesh. The fixed
        # world->base_link transform rotates robot and table together.  The
        # tabletop top surface is the base_link z=0 plane.
        "position": (
            inboard_distance,
            0.0,
            base_bottom_z - top_clearance - height / 2.0,
        ),
        "yaw": 0.0,
        "color": (0.18, 0.18, 0.18, 1.0),
    }


def parse_disabled_collision_pairs(
    semantic_description: str,
) -> list[tuple[str, str]]:
    """Read the current robot ACM entries from its semantic description."""
    if not semantic_description:
        return []
    root = ElementTree.fromstring(semantic_description)
    return [
        (entry.attrib["link1"], entry.attrib["link2"])
        for entry in root.findall("disable_collisions")
    ]


def create_worktable_planning_scene(
    geometry: dict[str, Any],
    disabled_pairs: Iterable[tuple[str, str]],
) -> PlanningScene:
    """Build a scene diff that permits only the fixed base/table contact."""
    table = CollisionObject()
    table.header.frame_id = "base_link"
    table.id = "fixed_worktable"
    table.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(geometry["dimensions"])

    pose = Pose()
    position = geometry["position"]
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.z = math.sin(geometry["yaw"] / 2.0)
    pose.orientation.w = math.cos(geometry["yaw"] / 2.0)
    table.primitives = [box]
    table.primitive_poses = [pose]

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [table]
    scene.link_padding = []
    for link_name in GRIPPER_COLLISION_LINKS:
        padding = LinkPadding()
        padding.link_name = link_name
        padding.padding = GRIPPER_COLLISION_PADDING_M
        scene.link_padding.append(padding)
    table_color = ObjectColor()
    table_color.id = table.id
    table_color.color.r = geometry["color"][0]
    table_color.color.g = geometry["color"][1]
    table_color.color.b = geometry["color"][2]
    table_color.color.a = geometry["color"][3]
    scene.object_colors = [table_color]
    allowed_pairs = set(disabled_pairs)
    allowed_pairs.add(("base_link", "fixed_worktable"))
    matrix = scene.allowed_collision_matrix
    matrix_names = {name for pair in allowed_pairs for name in pair}
    matrix.entry_names = sorted(matrix_names)
    matrix.entry_values = []
    for row_name in matrix.entry_names:
        row = [
            (row_name, column_name) in allowed_pairs
            or (column_name, row_name) in allowed_pairs
            for column_name in matrix.entry_names
        ]
        matrix.entry_values.append(AllowedCollisionEntry(enabled=row))
    return scene


class WorktableCollisionNode(Node):
    """Publish the fixed worktable and allowed collision matrix once."""

    def __init__(self) -> None:
        super().__init__("worktable_collision")
        self._publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.declare_parameter("length", 1.2)
        self.declare_parameter("width", 0.9)
        self.declare_parameter("height", 0.8)
        self.declare_parameter("mount_inset", 0.062)
        self.declare_parameter("wait_for_rviz", False)
        self.declare_parameter("robot_description_semantic", "")
        semantic_description = self.get_parameter(
            "robot_description_semantic"
        ).value
        self._disabled_pairs = parse_disabled_collision_pairs(semantic_description)
        if not self._disabled_pairs:
            raise RuntimeError(
                "robot_description_semantic has no collision matrix"
            )
        self._timer = self.create_timer(1.0, self.publish_worktable)
        self._logged = False

    def publish_worktable(self) -> None:
        if not worktable_publish_ready(
            self._publisher.get_subscription_count(),
            self.get_parameter("wait_for_rviz").value,
            self.count_subscribers("/monitored_planning_scene"),
        ):
            return
        geometry = create_worktable_geometry(
            length=self.get_parameter("length").value,
            width=self.get_parameter("width").value,
            height=self.get_parameter("height").value,
            mount_inset=self.get_parameter("mount_inset").value,
        )
        scene = create_worktable_planning_scene(geometry, self._disabled_pairs)
        self._publisher.publish(scene)

        if not self._logged:
            self.get_logger().info("fixed worktable added")
            self._logged = True
        # Publish once after MoveIt and the requested RViz consumer subscribe.
        # The scene then retains the object without overwriting later vision
        # updates to the same object ID.
        self._timer.cancel()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WorktableCollisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
