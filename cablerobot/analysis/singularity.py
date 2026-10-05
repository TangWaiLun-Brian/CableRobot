"""Small reference singularity metrics."""

from __future__ import annotations

import numpy as np

from cablerobot.model.state import RobotState


def cable_jacobian_rank(robot, state: RobotState, tolerance: float | None = None) -> int:
    return int(np.linalg.matrix_rank(robot.cable_jacobian(state), tol=tolerance))

