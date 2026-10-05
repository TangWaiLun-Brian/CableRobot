"""Generalized gravity loading for arbitrary body centers of mass."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.kinematics.jacobians import point_jacobian
from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def gravity_generalized_force(robot: "CableRobot", state: RobotState) -> NDArray[np.float64]:
    """Return the applied generalized force due to gravity, sum(J_com.T m g)."""
    robot.validate_state(state)
    robot.validate()
    generalized = np.zeros(robot.dof)
    for body in robot.bodies.values():
        if body.mass == 0.0 or body.fixed:
            continue
        jacobian = point_jacobian(robot, state, body.name, body.center_of_mass)
        generalized += jacobian.T @ (body.mass * robot.parameters.gravity)
    return generalized
