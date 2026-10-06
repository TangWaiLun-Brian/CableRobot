"""Central topology container for generic cable-driven multibody robots."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .body import Body
from .cable import Cable
from .frame import Frame
from .joint import Joint
from .state import RobotParameters, RobotState


@dataclass
class CableRobot:
    name: str
    parameters: RobotParameters = field(default_factory=RobotParameters)
    analysis_frames: tuple[str, ...] = ()
    bodies: dict[str, Body] = field(default_factory=dict, init=False)
    joints: list[Joint] = field(default_factory=list, init=False)
    frames: dict[str, Frame] = field(default_factory=dict, init=False)
    cables: list[Cable] = field(default_factory=list, init=False)

    def add_body(self, body: Body) -> Body:
        if body.name in self.bodies or body.name in self.frames:
            raise ValueError(f"duplicate body {body.name!r}")
        self.bodies[body.name] = body
        self.add_frame(Frame(name=body.name, body=body.name))
        return body

    def add_frame(self, frame: Frame) -> Frame:
        if frame.name in self.frames:
            raise ValueError(f"duplicate frame {frame.name!r}")
        if frame.body not in self.bodies:
            raise ValueError(f"unknown body {frame.body!r}")
        self.frames[frame.name] = frame
        return frame

    def add_joint(self, joint: Joint) -> Joint:
        if any(existing.name == joint.name for existing in self.joints):
            raise ValueError(f"duplicate joint {joint.name!r}")
        if joint.parent not in self.bodies or joint.child not in self.bodies:
            raise ValueError("joint parent and child bodies must be added first")
        if any(existing.child == joint.child for existing in self.joints):
            raise ValueError(f"body {joint.child!r} already has a parent joint")
        joint.q_start = self.dof
        self.joints.append(joint)
        return joint

    def add_cable(self, cable: Cable) -> Cable:
        if any(existing.name == cable.name for existing in self.cables):
            raise ValueError(f"duplicate cable {cable.name!r}")
        missing = [point.frame for point in cable.route.points if point.frame not in self.frames]
        if missing:
            raise ValueError(f"cable {cable.name!r} references unknown frames: {missing}")
        self.cables.append(cable)
        return cable

    @property
    def dof(self) -> int:
        return sum(joint.dof for joint in self.joints)

    @property
    def cable_count(self) -> int:
        return len(self.cables)

    def zero_state(self) -> RobotState:
        return RobotState(np.zeros(self.dof))

    def validate_state(self, state: RobotState) -> None:
        for name in ("q", "qd"):
            value = np.asarray(getattr(state, name), dtype=float)
            if value.shape != (self.dof,) or not np.all(np.isfinite(value)):
                raise ValueError(f"state.{name} must be finite with shape ({self.dof},); robot requires {self.dof} coordinates")

    def validate(self) -> None:
        if not self.bodies:
            raise ValueError("robot has no bodies")
        for name, body in self.bodies.items():
            if name != body.name or not isinstance(body.fixed, bool):
                raise ValueError("body dictionary keys must match names; fixed must be boolean")
            frame = self.frames.get(name)
            if (frame is None or frame.body != name or np.shape(frame.T_body_frame) != (4, 4)
                    or not np.allclose(frame.T_body_frame, np.eye(4), atol=1e-12, rtol=0)):
                raise ValueError("same-named body frames must remain identity on their own body; add a separate offset frame")
        for name, frame in self.frames.items():
            if name != frame.name or frame.body not in self.bodies:
                raise ValueError("frame dictionary keys must match names and reference known bodies")
        joint_names, children = set(), set()
        coordinate_start = 0
        for joint in self.joints:
            if joint.name in joint_names or joint.child in children:
                raise ValueError("joint names and child bodies must be unique")
            if joint.parent not in self.bodies or joint.child not in self.bodies or joint.parent == joint.child:
                raise ValueError("joint must reference distinct known parent and child bodies")
            if joint.q_start != coordinate_start:
                raise ValueError("joint coordinate blocks must follow insertion order; do not reorder joints directly")
            joint_names.add(joint.name)
            children.add(joint.child)
            coordinate_start += joint.dof
        cable_names = set()
        for cable in self.cables:
            if cable.name in cable_names:
                raise ValueError("cable names must be unique")
            cable_names.add(cable.name)
            if len(cable.route.points) < 2 or any(point.frame not in self.frames for point in cable.route.points):
                raise ValueError("cable routes require at least two points on known frames")
            cable.validate()
        if any(name not in self.frames for name in self.analysis_frames):
            raise ValueError("analysis_frames references an unknown frame")
        roots = [name for name in self.bodies if name not in children]
        if not roots:
            raise ValueError("multibody graph has no root")
        if any(not self.bodies[name].fixed for name in roots):
            raise ValueError("root bodies must be fixed in world; use a floating joint for a moving root")
        resolved = set(roots)
        stationary = set(roots)
        remaining = list(self.joints)
        while remaining:
            progress = False
            for joint in remaining[:]:
                if joint.parent in resolved:
                    is_stationary = joint.parent in stationary and joint.dof == 0
                    if self.bodies[joint.child].fixed and not is_stationary:
                        raise ValueError(f"fixed body {joint.child!r} has a moving ancestor")
                    if is_stationary:
                        stationary.add(joint.child)
                    resolved.add(joint.child)
                    remaining.remove(joint)
                    progress = True
            if not progress:
                raise ValueError("multibody graph is disconnected or cyclic")
        if resolved != set(self.bodies):
            raise ValueError("not every body is connected to a root")

    def body_transforms(self, state: RobotState) -> dict[str, NDArray[np.float64]]:
        from cablerobot.kinematics.bodies import body_transforms

        return body_transforms(self, state)

    def frame_transform(self, frame: str, state: RobotState) -> NDArray[np.float64]:
        from cablerobot.kinematics.bodies import frame_transform

        return frame_transform(self, frame, state)

    def cable_lengths(self, state: RobotState) -> NDArray[np.float64]:
        from cablerobot.kinematics.cables import cable_lengths

        return cable_lengths(self, state)

    def cable_jacobian(self, state: RobotState, *, method: str | None = None) -> NDArray[np.float64]:
        from cablerobot.kinematics.jacobians import cable_jacobian

        return cable_jacobian(self, state, method=method)

    def cable_force_matrix(self, state: RobotState) -> NDArray[np.float64]:
        return -self.cable_jacobian(state).T

    def tension_bounds(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return (
            np.array([cable.tension_min for cable in self.cables], dtype=float),
            np.array([cable.tension_max for cable in self.cables], dtype=float),
        )
