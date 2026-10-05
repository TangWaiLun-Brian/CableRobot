"""Reference generalized cable manipulability metric."""

from __future__ import annotations

import numpy as np


def cable_manipulability(robot, state) -> float:
    J = robot.cable_jacobian(state)
    if J.shape[0] < J.shape[1]:
        return 0.0
    singular_values = np.linalg.svd(J, compute_uv=False)
    return float(np.prod(singular_values))
