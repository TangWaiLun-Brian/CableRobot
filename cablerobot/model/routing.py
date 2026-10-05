"""Cable attachments and declared routes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(slots=True)
class Attachment:
    frame: str
    point: NDArray[np.float64]
    label: str | None = None

    def __post_init__(self) -> None:
        self.point = np.asarray(self.point, dtype=float)
        if not self.frame:
            raise ValueError("attachment frame is required")
        if self.point.shape != (3,):
            raise ValueError("attachment point must have shape (3,)")
        if not np.all(np.isfinite(self.point)):
            raise ValueError("attachment point must be finite")


@dataclass(slots=True)
class CableRoute:
    points: tuple[Attachment, ...]

    def __init__(self, points: list[Attachment] | tuple[Attachment, ...]):
        self.points = tuple(points)
        if len(self.points) < 2:
            raise ValueError("a cable route requires at least two points")
