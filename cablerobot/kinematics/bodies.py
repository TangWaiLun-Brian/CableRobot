"""Forward body and frame kinematics."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.model.state import RobotState
from cablerobot.model.transforms import transform_point

if False:  # pragma: no cover - type checking without a runtime cycle
    from cablerobot.model.robot import CableRobot


def body_transforms(robot: "CableRobot", state: RobotState) -> dict[str, NDArray[np.float64]]:
    """Return T_world_body for every body in a tree or forest."""
    robot.validate_state(state)
    robot.validate()
    children = {joint.child for joint in robot.joints}
    transforms = {name: np.eye(4) for name in robot.bodies if name not in children}
    remaining = list(robot.joints)
    while remaining:
        progress = False
        for joint in remaining[:]:
            if joint.parent not in transforms:
                continue
            start = int(joint.q_start or 0)
            q_joint = state.q[start : start + joint.dof]
            transforms[joint.child] = (
                transforms[joint.parent]
                @ joint.T_parent_joint
                @ joint.motion_transform(q_joint)
                @ joint.T_joint_child
            )
            remaining.remove(joint)
            progress = True
        if not progress:
            raise ValueError("cannot resolve body transforms; validate the multibody graph")
    return transforms


def frame_transform(robot: "CableRobot", frame: str, state: RobotState) -> NDArray[np.float64]:
    if frame not in robot.frames:
        raise KeyError(f"unknown frame {frame!r}")
    framespec = robot.frames[frame]
    return body_transforms(robot, state)[framespec.body] @ framespec.T_body_frame


def frame_position(robot: "CableRobot", frame: str, state: RobotState) -> NDArray[np.float64]:
    return frame_transform(robot, frame, state)[:3, 3]


def point_position(robot: "CableRobot", frame: str, point: NDArray[np.float64], state: RobotState) -> NDArray[np.float64]:
    return transform_point(frame_transform(robot, frame, state), point)
