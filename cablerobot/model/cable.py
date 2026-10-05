"""Cable model for straight, massless, tension-only segments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .routing import CableRoute


@dataclass(slots=True)
class Cable:
    name: str
    route: CableRoute
    tension_min: float = 0.0
    tension_max: float = np.inf
    stiffness: float | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("cable name cannot be empty")
        if not np.isfinite(self.tension_min) or self.tension_min < 0.0:
            raise ValueError("cables are tension-only: tension_min must be nonnegative")
        if np.isnan(self.tension_max) or self.tension_max < self.tension_min:
            raise ValueError("tension_max must be at least tension_min")
        if self.stiffness is not None and (not np.isfinite(self.stiffness) or self.stiffness <= 0.0):
            raise ValueError("optional stiffness must be positive and finite")
