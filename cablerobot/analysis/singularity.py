"""Small reference singularity metrics."""

from __future__ import annotations

import numpy as np

from cablerobot.model.state import RobotState


def cable_jacobian_rank(robot, state: RobotState, tolerance: float | None = None) -> int:
    matrix = robot.cable_jacobian(state)
    return 0 if matrix.size == 0 else int(np.linalg.matrix_rank(matrix, tol=tolerance))
