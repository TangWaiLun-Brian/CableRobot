"""Always-available NumPy numerical backend."""

from __future__ import annotations

from cablerobot.allocation.tension import TensionAllocationResult, allocate_tensions
from .base import TensionProblem


class NumPyBackend:
    name = "numpy"

    def solve_tension_problem(self, problem: TensionProblem) -> TensionAllocationResult:
        return allocate_tensions(problem.force_matrix, problem.target, problem.lower, problem.upper, tolerance=problem.tolerance)

