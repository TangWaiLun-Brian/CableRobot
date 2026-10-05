"""Available generalized-force sets under bounded tension."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np
from numpy.typing import NDArray

from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


@dataclass(frozen=True, slots=True)
class GeneralizedForceSet:
    force_matrix: NDArray[np.float64]
    tension_min: NDArray[np.float64]
    tension_max: NDArray[np.float64]
    vertices: NDArray[np.float64]


def available_generalized_force_set(robot: "CableRobot", state: RobotState, *, max_cables: int = 16) -> GeneralizedForceSet:
    matrix = robot.cable_force_matrix(state)
    lower, upper = robot.tension_bounds()
    if np.any(~np.isfinite(upper)):
        raise ValueError("finite upper tension bounds are required for a bounded force set")
    if robot.cable_count > max_cables:
        vertices = np.empty((0, robot.dof))
    else:
        vertices = np.vstack([matrix @ np.where(np.array(bits, dtype=bool), upper, lower) for bits in product((0, 1), repeat=robot.cable_count)])
    return GeneralizedForceSet(matrix, lower, upper, vertices)


def cable_wrench_matrix(robot: "CableRobot", state: RobotState, selected_frame: str) -> NDArray[np.float64]:
    """Per-body [force; moment] in world axes, about an explicitly selected frame.

    Includes every cable segment incident on that frame's body, including routing
    points in the middle of a cable. This is a local body wrench, not all-system
    generalized force for a serial mechanism.
    """
    from cablerobot.kinematics.cables import cable_route_points

    body_name = robot.frames[selected_frame].body
    origin = robot.frame_transform(selected_frame, state)[:3, 3]
    matrix = np.zeros((6, robot.cable_count))
    for column, cable in enumerate(robot.cables):
        points = cable_route_points(robot, cable, state)
        for segment in range(len(points) - 1):
            delta = points[segment + 1] - points[segment]
            length = float(np.linalg.norm(delta))
            if length < 1e-12:
                raise ValueError("wrench mapping is undefined for a zero-length segment")
            direction = delta / length
            for index, force in ((segment, direction), (segment + 1, -direction)):
                frame_name = cable.route.points[index].frame
                if robot.frames[frame_name].body == body_name:
                    matrix[:3, column] += force
                    matrix[3:, column] += np.cross(points[index] - origin, force)
    return matrix
