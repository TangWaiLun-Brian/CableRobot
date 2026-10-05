"""Rigid-body data."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(slots=True)
class Body:
    name: str
    mass: float = 0.0
    center_of_mass: NDArray[np.float64] = field(default_factory=lambda: np.zeros(3))
    inertia: NDArray[np.float64] = field(default_factory=lambda: np.zeros((3, 3)))
    fixed: bool = False
    visual_size: float = 0.12

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("body name cannot be empty")
        if not np.isfinite(self.mass) or self.mass < 0.0:
            raise ValueError("body mass cannot be negative")
        self.center_of_mass = np.asarray(self.center_of_mass, dtype=float)
        self.inertia = np.asarray(self.inertia, dtype=float)
        if self.center_of_mass.shape != (3,):
            raise ValueError("center_of_mass must have shape (3,)")
        if self.inertia.shape != (3, 3):
            raise ValueError("inertia must have shape (3, 3)")
        if not np.all(np.isfinite(self.center_of_mass)) or not np.all(np.isfinite(self.inertia)):
            raise ValueError("body inertial data must be finite")
        if not np.allclose(self.inertia, self.inertia.T, atol=1e-12) or np.min(np.linalg.eigvalsh(self.inertia)) < -1e-12:
            raise ValueError("inertia must be symmetric positive semidefinite")
        if not np.isfinite(self.visual_size) or self.visual_size <= 0.0:
            raise ValueError("visual_size must be positive and finite")
