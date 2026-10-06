"""Cable route geometry."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.kinematics.bodies import _KinematicsContext
from cablerobot.model.cable import Cable
from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def cable_route_points(robot: "CableRobot", cable: Cable, state: RobotState) -> NDArray[np.float64]:
    return _route_points(_KinematicsContext(robot, state), cable)


def _route_points(context, cable):
    return np.vstack([context.point(point.frame, point.point) for point in cable.route.points])


def cable_length(robot: "CableRobot", cable: Cable, state: RobotState) -> float:
    points = cable_route_points(robot, cable, state)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def cable_lengths(robot: "CableRobot", state: RobotState) -> NDArray[np.float64]:
    context = _KinematicsContext(robot, state)
    return np.array([np.linalg.norm(np.diff(_route_points(context, cable), axis=0), axis=1).sum()
                     for cable in robot.cables], dtype=float)


def _cable_lengths_and_jacobian(robot: "CableRobot", state: RobotState):
    """Differentiate all straight routed segments using one body/frame evaluation."""
    context = _KinematicsContext(robot, state, derivatives=True)
    lengths = np.empty(robot.cable_count)
    jacobian = np.zeros((robot.cable_count, robot.dof))
    for row, cable in enumerate(robot.cables):
        positions = _route_points(context, cable)
        segments = np.diff(positions, axis=0)
        segment_lengths = np.linalg.norm(segments, axis=1)
        if np.any(segment_lengths < 1e-12):
            raise ValueError(f"cable {cable.name!r} contains a zero-length segment; its Jacobian is undefined")
        points_jacobian = np.stack([context.point_jacobian(point.frame, point.point, position=position)
                                    for point, position in zip(cable.route.points, positions)])
        lengths[row] = segment_lengths.sum()
        jacobian[row] = np.einsum("si,sij->j", segments / segment_lengths[:, None], np.diff(points_jacobian, axis=0))
    return lengths, jacobian
