"""Robot state and parameter values."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass(slots=True)
class RobotState:
    q: NDArray[np.float64]
    qd: NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        self.q = np.asarray(self.q, dtype=float)
        if self.q.ndim != 1:
            raise ValueError("q must be one-dimensional")
        if not np.all(np.isfinite(self.q)):
            raise ValueError("q must be finite")
        if self.qd is None:
            self.qd = np.zeros_like(self.q)
        else:
            self.qd = np.asarray(self.qd, dtype=float)
            if self.qd.shape != self.q.shape:
                raise ValueError("qd must have the same shape as q")
        if not np.all(np.isfinite(self.qd)):
            raise ValueError("qd must be finite")


@dataclass(slots=True)
class RobotParameters:
    gravity: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    finite_difference_step: float = 1e-6

    def __post_init__(self) -> None:
        self.gravity = np.asarray(self.gravity, dtype=float)
        if self.gravity.shape != (3,):
            raise ValueError("gravity must have shape (3,)")
        if not np.all(np.isfinite(self.gravity)):
            raise ValueError("gravity must be finite")
        if not np.isfinite(self.finite_difference_step) or self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
