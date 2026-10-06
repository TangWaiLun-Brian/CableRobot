"""Forward body and frame kinematics."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from cablerobot.model.state import RobotState
from cablerobot.model.transforms import skew, transform_point
from cablerobot.model.joint import JointType

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


def _so3_left_jacobian(rotvec):
    """World/left-trivialized angular rate of Exp(rotvec), in joint axes."""
    squared = float(rotvec @ rotvec)
    if squared < 1e-8:
        a = 0.5 - squared / 24 + squared**2 / 720
        b = 1 / 6 - squared / 120 + squared**2 / 5040
    else:
        theta = np.sqrt(squared)
        a = (1 - np.cos(theta)) / squared
        b = (theta - np.sin(theta)) / (squared * theta)
    cross = skew(rotvec)
    return np.eye(3) + a * cross + b * cross @ cross


class _KinematicsContext:
    """Validated call-local transform/derivative reuse; never cached on a model.

    Positions follow the original body_transforms exactly. Angular Jacobians map
    coordinate rates to WORLD angular velocity, not rotation-vector rates directly.
    """

    def __init__(self, robot: "CableRobot", state: RobotState, *, derivatives=False):
        self.robot = robot
        self.transforms = body_transforms(robot, state)
        self.frames = {}
        self.linear, self.angular = {}, {}
        if not derivatives:
            return
        children = {joint.child for joint in robot.joints}
        for name in robot.bodies:
            if name not in children:
                self.linear[name] = np.zeros((3, robot.dof))
                self.angular[name] = np.zeros((3, robot.dof))
        remaining = list(robot.joints)
        while remaining:
            before = len(remaining)
            for joint in remaining[:]:
                if joint.parent not in self.linear:
                    continue
                parent = self.transforms[joint.parent]
                child = self.transforms[joint.child]
                linear = self.linear[joint.parent] - skew(child[:3, 3] - parent[:3, 3]) @ self.angular[joint.parent]
                angular = self.angular[joint.parent].copy()
                pre = parent @ joint.T_parent_joint
                start = int(joint.q_start or 0)
                if joint.joint_type is JointType.REVOLUTE:
                    axis = pre[:3, :3] @ joint.axis
                    angular[:, start] += axis
                    linear[:, start] += np.cross(axis, child[:3, 3] - pre[:3, 3])
                elif joint.joint_type is JointType.PRISMATIC:
                    linear[:, start] += pre[:3, :3] @ joint.axis
                elif joint.joint_type is JointType.FLOATING:
                    q = state.q[start:start+6]
                    axes = pre[:3, :3] @ _so3_left_jacobian(q[3:])
                    origin = pre[:3, 3] + pre[:3, :3] @ q[:3]
                    linear[:, start:start+3] += pre[:3, :3]
                    linear[:, start+3:start+6] -= skew(child[:3, 3] - origin) @ axes
                    angular[:, start+3:start+6] += axes
                self.linear[joint.child], self.angular[joint.child] = linear, angular
                remaining.remove(joint)
            if len(remaining) == before:
                raise ValueError("cannot resolve body derivatives; validate the multibody graph")

    def point(self, frame, point):
        if frame not in self.frames:
            specification = self.robot.frames[frame]
            self.frames[frame] = self.transforms[specification.body] @ specification.T_body_frame
        return transform_point(self.frames[frame], point)

    def point_jacobian(self, frame, point, *, position=None):
        body = self.robot.frames[frame].body
        position = self.point(frame, point) if position is None else position
        return self.linear[body] - skew(position - self.transforms[body][:3, 3]) @ self.angular[body]
