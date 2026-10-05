"""Lazy optional MATLAB adapter boundary."""

from __future__ import annotations

import importlib.util
from typing import NoReturn

from cablerobot.allocation.tension import TensionAllocationResult
from .base import BackendUnavailableError, TensionProblem


class MatlabBackend:
    name = "matlab"

    @staticmethod
    def available() -> bool:
        try:
            return importlib.util.find_spec("matlab.engine") is not None
        except (ModuleNotFoundError, ValueError):
            return False

    def require_solver(self) -> NoReturn:
        """Reject explicit selection until an engine-backed solver exists."""
        if not self.available():
            raise BackendUnavailableError(
                "MATLAB Engine for Python is not installed. The core package and NumPy backend remain fully usable."
            )
        raise BackendUnavailableError(
            "MATLAB Engine was detected, but milestone 1 provides only the adapter boundary; configure a MATLAB solver implementation first."
        )

    def solve_tension_problem(self, problem: TensionProblem) -> TensionAllocationResult:
        self.require_solver()
