"""Static force balance."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from cablerobot.model.state import RobotState
from .gravity import gravity_generalized_force

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def cable_generalized_force(robot: "CableRobot", state: RobotState, tensions: ArrayLike) -> NDArray[np.float64]:
    values = np.asarray(tensions, dtype=float)
    if values.shape != (robot.cable_count,):
        raise ValueError("tensions has the wrong shape")
    return robot.cable_force_matrix(state) @ values


def equilibrium_residual(
    robot: "CableRobot",
    state: RobotState,
    tensions: ArrayLike,
    external_load: ArrayLike | None = None,
    *,
    include_gravity: bool = True,
) -> NDArray[np.float64]:
    residual = cable_generalized_force(robot, state, tensions)
    if include_gravity:
        residual = residual + gravity_generalized_force(robot, state)
    if external_load is not None:
        load = np.asarray(external_load, dtype=float)
        if load.shape != (robot.dof,):
            raise ValueError("external_load has the wrong shape")
        residual = residual + load
    return residual

