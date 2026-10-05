"""Backend-neutral numerical problem and result types."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from cablerobot.allocation.tension import TensionAllocationResult


class BackendUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class TensionProblem:
    force_matrix: NDArray[np.float64]
    target: NDArray[np.float64]
    lower: NDArray[np.float64]
    upper: NDArray[np.float64]
    tolerance: float = 1e-8


class AnalysisBackend(Protocol):
    name: str

    def solve_tension_problem(self, problem: TensionProblem) -> TensionAllocationResult:
        ...
