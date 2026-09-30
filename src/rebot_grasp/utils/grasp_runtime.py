"""Resolve GraspNet runtime settings with predictable CLI precedence."""

from __future__ import annotations

from argparse import Namespace
from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class GraspNetRuntime:
    checkpoint: str
    num_point: int
    num_view: int
    collision_thresh: float
    voxel_size: float
    min_depth: float
    max_depth: float
    target_class: Optional[str]
    target_margin_px: int
    target_expand_ratio: float


def _cli_or_config(
    args: Namespace,
    cli_name: str,
    config: dict[str, Any],
    config_name: str,
    default: Any,
) -> Any:
    cli_value = getattr(args, cli_name, None)
    if cli_value is not None:
        return cli_value
    return config.get(config_name, default)


def resolve_graspnet_runtime(
    cfg: dict[str, Any],
    args: Namespace,
) -> GraspNetRuntime:
    graspnet = cfg.get("graspnet") or {}
    runtime = GraspNetRuntime(
        checkpoint=str(
            _cli_or_config(
                args,
                "checkpoint",
                graspnet,
                "checkpoint",
                "checkpoint-rs.tar",
            )
        ),
        num_point=int(
            _cli_or_config(args, "num_point", graspnet, "num_point", 20000)
        ),
        num_view=int(
            _cli_or_config(args, "num_view", graspnet, "num_view", 300)
        ),
        collision_thresh=float(
            _cli_or_config(
                args,
                "collision_thresh",
                graspnet,
                "collision_thresh",
                0.01,
            )
        ),
        voxel_size=float(
            _cli_or_config(
                args,
                "voxel_size",
                graspnet,
                "voxel_size",
                0.01,
            )
        ),
        min_depth=float(
            _cli_or_config(args, "min_depth", graspnet, "min_depth", 0.05)
        ),
        max_depth=float(
            _cli_or_config(args, "max_depth", graspnet, "max_depth", 2.0)
        ),
        target_class=_cli_or_config(
            args,
            "target_class",
            graspnet,
            "target_class",
            None,
        ),
        target_margin_px=int(
            _cli_or_config(
                args,
                "target_margin_px",
                graspnet,
                "target_margin_px",
                12,
            )
        ),
        target_expand_ratio=float(
            _cli_or_config(
                args,
                "target_expand_ratio",
                graspnet,
                "target_expand_ratio",
                1.0,
            )
        ),
    )
    if runtime.num_point <= 0:
        raise ValueError("graspnet.num_point must be positive")
    if runtime.num_view <= 0:
        raise ValueError("graspnet.num_view must be positive")
    if runtime.voxel_size <= 0.0:
        raise ValueError("graspnet.voxel_size must be positive")
    if not 0.0 <= runtime.min_depth < runtime.max_depth:
        raise ValueError("graspnet depth range must satisfy 0 <= min_depth < max_depth")
    if runtime.target_margin_px < 0:
        raise ValueError("graspnet.target_margin_px must be non-negative")
    if runtime.target_expand_ratio < 1.0:
        raise ValueError("graspnet.target_expand_ratio must be at least 1.0")
    return runtime
