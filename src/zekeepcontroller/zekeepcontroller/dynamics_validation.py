from __future__ import annotations

from collections.abc import Iterable

import numpy as np


def validate_dynamics_model(model, joint_names: Iterable[str]) -> None:
    """Validate positive, finite mass properties for every controlled joint."""
    for name in joint_names:
        joint_id = int(model.getJointId(name))
        if joint_id <= 0 or joint_id >= int(model.njoints):
            raise ValueError(f"missing dynamics for {name}")

        body_inertia = model.inertias[joint_id]
        mass = float(body_inertia.mass)
        if not np.isfinite(mass) or mass <= 0.0:
            raise ValueError(f"{name} mass must be positive and finite")

        center = np.asarray(body_inertia.lever, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError(f"{name} center of mass must contain three finite values")

        tensor = np.asarray(body_inertia.inertia, dtype=np.float64)
        if tensor.shape != (3, 3) or not np.all(np.isfinite(tensor)):
            raise ValueError(f"{name} inertia tensor must be a finite 3x3 matrix")
        if not np.allclose(tensor, tensor.T, rtol=1e-9, atol=1e-12):
            raise ValueError(f"{name} inertia tensor must be symmetric")
        if np.min(np.linalg.eigvalsh(tensor)) <= 0.0:
            raise ValueError(f"{name} inertia tensor must be positive definite")
