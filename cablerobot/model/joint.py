"""Joint definitions for a tree-structured multibody system."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from numpy.typing import NDArray

from .transforms import rotation_about_axis, rotation_from_rotvec, transform, validate_transform


class JointType(str, Enum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FLOATING = "floating"


_DOF = {JointType.FIXED: 0, JointType.REVOLUTE: 1, JointType.PRISMATIC: 1, JointType.FLOATING: 6}


@dataclass(slots=True)
class Joint:
    name: str
    parent: str
    child: str
    joint_type: JointType | str
    axis: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    T_parent_joint: NDArray[np.float64] = field(default_factory=lambda: np.eye(4))
    T_joint_child: NDArray[np.float64] = field(default_factory=lambda: np.eye(4))
    q_start: int | None = None

    def __post_init__(self) -> None:
        if not self.name or not self.parent or not self.child or self.parent == self.child:
            raise ValueError("joint requires a name and distinct parent/child bodies")
        self.joint_type = JointType(self.joint_type)
        self.axis = np.asarray(self.axis, dtype=float)
        if self.axis.shape != (3,):
            raise ValueError("joint axis must have shape (3,)")
        if not np.all(np.isfinite(self.axis)):
            raise ValueError("joint axis must be finite")
        if self.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
            norm = float(np.linalg.norm(self.axis))
            if norm == 0.0:
                raise ValueError("joint axis must be nonzero")
            self.axis = self.axis / norm
        self.T_parent_joint = validate_transform(self.T_parent_joint, "T_parent_joint")
        self.T_joint_child = validate_transform(self.T_joint_child, "T_joint_child")

    @property
    def dof(self) -> int:
        return _DOF[self.joint_type]

    def motion_transform(self, coordinates: NDArray[np.float64]) -> NDArray[np.float64]:
        if coordinates.shape != (self.dof,):
            raise ValueError(f"joint {self.name!r} expects {self.dof} coordinates")
        if self.joint_type is JointType.FIXED:
            return np.eye(4)
        if self.joint_type is JointType.REVOLUTE:
            return transform(rotation_about_axis(self.axis, float(coordinates[0])))
        if self.joint_type is JointType.PRISMATIC:
            return transform(translation=self.axis * float(coordinates[0]))
        return transform(rotation_from_rotvec(coordinates[3:]), coordinates[:3])
