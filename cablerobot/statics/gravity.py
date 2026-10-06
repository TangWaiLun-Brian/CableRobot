"""Generalized gravity loading for arbitrary body centers of mass."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.kinematics.jacobians import point_jacobian
from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def gravity_generalized_force(robot: "CableRobot", state: RobotState, *,
                              jacobian_method: str = "analytic") -> NDArray[np.float64]:
    """Applied sum(J_com.T m g); numerical COM derivatives remain selectable."""
    from cablerobot.kinematics.bodies import _KinematicsContext
    from cablerobot.kinematics.jacobians import _method

    selected = _method(jacobian_method, None)
    robot.validate_state(state)
    robot.validate()
    generalized = np.zeros(robot.dof)
    context = _KinematicsContext(robot, state, derivatives=True) if selected == "analytic" else None
    for body in robot.bodies.values():
        if body.mass == 0.0 or body.fixed:
            continue
        jacobian = (context.point_jacobian(body.name, body.center_of_mass) if context is not None
                    else point_jacobian(robot, state, body.name, body.center_of_mass, method="finite_difference"))
        generalized += jacobian.T @ (body.mass * robot.parameters.gravity)
    return generalized
