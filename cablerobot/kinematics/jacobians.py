"""Generalized Jacobians for arbitrary cable routes and body points."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def numerical_jacobian(function: Callable[[NDArray[np.float64]], NDArray[np.float64]], q: NDArray[np.float64], step: float) -> NDArray[np.float64]:
    if not np.isfinite(step) or step <= 0.0:
        raise ValueError("finite difference step must be positive and finite")
    y0 = np.atleast_1d(function(q))
    jacobian = np.empty((y0.size, q.size), dtype=float)
    for column in range(q.size):
        delta = np.zeros_like(q)
        delta[column] = step
        jacobian[:, column] = (function(q + delta) - function(q - delta)) / (2.0 * step)
    return jacobian


def _method(method, step):
    selected = ("analytic" if step is None else "finite_difference") if method is None else method
    if selected not in ("analytic", "finite_difference"):
        raise ValueError("Jacobian method must be 'analytic' or 'finite_difference'")
    if selected == "analytic" and step is not None:
        raise ValueError("a finite-difference step cannot be supplied to the analytic method")
    return selected


def cable_jacobian(robot: "CableRobot", state: RobotState, step: float | None = None,
                   *, method: str | None = None) -> NDArray[np.float64]:
    """Return dl/dq for all supported body-fixed straight routed segments.

    Default analytic; an explicit step or method='finite_difference' retains the
    central-difference reference. Floating derivatives respect Exp(rotvec) rates.
    """
    selected = _method(method, step)
    if selected == "analytic":
        from .cables import _cable_lengths_and_jacobian
        return _cable_lengths_and_jacobian(robot, state)[1]
    return _finite_difference_cable_jacobian(robot, state, step)


def _finite_difference_cable_jacobian(robot, state, step):
    """Retained foundation central-difference derivative and degeneracy policy."""
    robot.validate_state(state)
    from .cables import cable_route_points

    for cable in robot.cables:
        if np.any(np.linalg.norm(np.diff(cable_route_points(robot, cable, state), axis=0), axis=1) < 1e-12):
            raise ValueError(f"cable {cable.name!r} contains a zero-length segment; its Jacobian is undefined")
    h = robot.parameters.finite_difference_step if step is None else step
    return numerical_jacobian(lambda q: robot.cable_lengths(RobotState(q)), state.q, h)


def point_jacobian(robot: "CableRobot", state: RobotState, frame: str, point: NDArray[np.float64],
                   step: float | None = None, *, method: str | None = None) -> NDArray[np.float64]:
    from cablerobot.kinematics.bodies import point_position, _KinematicsContext

    if _method(method, step) == "analytic":
        return _KinematicsContext(robot, state, derivatives=True).point_jacobian(frame, point)
    h = robot.parameters.finite_difference_step if step is None else step
    return numerical_jacobian(lambda q: point_position(robot, frame, point, RobotState(q)), state.q, h)
