"""Frames rigidly attached to bodies."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .transforms import validate_transform


@dataclass(slots=True)
class Frame:
    name: str
    body: str
    T_body_frame: NDArray[np.float64] = field(default_factory=lambda: np.eye(4))

    def __post_init__(self) -> None:
        if not self.name or not self.body:
            raise ValueError("frame name and body are required")
        self.T_body_frame = validate_transform(self.T_body_frame, "T_body_frame")

