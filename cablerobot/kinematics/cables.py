"""Cable route geometry."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.kinematics.bodies import point_position
from cablerobot.model.cable import Cable
from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


def cable_route_points(robot: "CableRobot", cable: Cable, state: RobotState) -> NDArray[np.float64]:
    return np.vstack([point_position(robot, point.frame, point.point, state) for point in cable.route.points])


def cable_length(robot: "CableRobot", cable: Cable, state: RobotState) -> float:
    points = cable_route_points(robot, cable, state)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def cable_lengths(robot: "CableRobot", state: RobotState) -> NDArray[np.float64]:
    robot.validate_state(state)
    robot.validate()
    return np.array([cable_length(robot, cable, state) for cable in robot.cables])
