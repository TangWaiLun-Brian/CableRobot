"""Twist analysis extension point.

Twists are meaningful only after selecting a body or frame. Full bounded twist
sets are deliberately deferred beyond milestone 1.
"""

import numpy as np

from cablerobot.model.state import RobotState


def frame_twist_jacobian(robot, state: RobotState, frame: str):
    """Return world [linear velocity of frame origin; angular velocity] per q_dot."""
    require_selected_frame(frame)
    robot.validate_state(state)
    h = robot.parameters.finite_difference_step
    T = robot.frame_transform(frame, state)
    jacobian = np.zeros((6, robot.dof))
    for column in range(robot.dof):
        delta = np.zeros(robot.dof)
        delta[column] = h
        plus = robot.frame_transform(frame, RobotState(state.q + delta))
        minus = robot.frame_transform(frame, RobotState(state.q - delta))
        jacobian[:3, column] = (plus[:3, 3] - minus[:3, 3]) / (2.0 * h)
        omega = ((plus[:3, :3] - minus[:3, :3]) / (2.0 * h)) @ T[:3, :3].T
        jacobian[3:, column] = [omega[2, 1], omega[0, 2], omega[1, 0]]
    return jacobian


def require_selected_frame(frame: str | None) -> str:
    if not frame:
        raise ValueError("twist analysis requires a selected body or frame")
    return frame
