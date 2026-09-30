#!/usr/bin/env bash
set -euo pipefail

action=${1:-add}

case "$action" in
  add)
    scene='{is_diff: true, world: {collision_objects: [{header: {frame_id: base_link}, id: training_box, operation: [0], primitives: [{type: 1, dimensions: [0.20, 0.10, 0.50]}], primitive_poses: [{position: {x: 0.40, y: 0.0, z: 0.10}, orientation: {w: 1.0}}]}]}}'
    ;;
  remove)
    scene='{is_diff: true, world: {collision_objects: [{header: {frame_id: base_link}, id: training_box, operation: [1]}]}}'
    ;;
  *)
    echo "Usage: $(basename "$0") [add|remove]" >&2
    exit 2
    ;;
esac

ros2 topic pub --once /planning_scene moveit_msgs/msg/PlanningScene "$scene"
