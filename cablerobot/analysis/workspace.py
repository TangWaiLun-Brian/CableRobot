"""Backend-neutral analysis requests."""

from __future__ import annotations

from dataclasses import dataclass

from numpy.typing import NDArray

from cablerobot.allocation.tension import TensionAllocationResult
from cablerobot.backends import TensionProblem, get_backend
from cablerobot.model.state import RobotState
from .wrench import GeneralizedForceSet, available_generalized_force_set

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


@dataclass(frozen=True, slots=True)
class WrenchAnalysisResult:
    backend: str
    force_set: GeneralizedForceSet
    allocation: TensionAllocationResult | None = None


def analyze_wrench_workspace(
    robot: "CableRobot",
    state: RobotState,
    *,
    backend: str = "numpy",
    target_generalized_force: NDArray | None = None,
) -> WrenchAnalysisResult:
    force_set = available_generalized_force_set(robot, state)
    allocation = None
    if target_generalized_force is not None:
        problem = TensionProblem(force_set.force_matrix, target_generalized_force, force_set.tension_min, force_set.tension_max)
        allocation = get_backend(backend).solve_tension_problem(problem)
    else:
        get_backend(backend)  # validate selection without doing unnecessary work
    return WrenchAnalysisResult(backend, force_set, allocation)

