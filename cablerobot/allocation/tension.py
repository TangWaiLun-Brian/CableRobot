"""Dependency-free bounded cable-tension allocation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from itertools import product

import numpy as np
from numpy.typing import ArrayLike, NDArray

from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


class AllocationStatus(str, Enum):
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    NUMERICAL_FAILURE = "numerical_failure"
    NUMERICALLY_UNRESOLVED = "numerically_unresolved"


@dataclass(frozen=True, slots=True)
class TensionAllocationResult:
    status: AllocationStatus
    tensions: NDArray[np.float64]
    achieved_generalized_force: NDArray[np.float64]
    target_generalized_force: NDArray[np.float64]
    residual: NDArray[np.float64]
    residual_norm: float
    message: str

    @property
    def feasible(self) -> bool:
        return self.status is AllocationStatus.FEASIBLE


def _enumerated_box_least_squares(
    matrix: NDArray[np.float64],
    target: NDArray[np.float64],
    lower: NDArray[np.float64],
    upper: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Solve a small box-constrained least-squares problem by active sets."""
    count = matrix.shape[1]
    states = [(0, 1, 2) if np.isfinite(upper[i]) else (0, 1) for i in range(count)]
    best = lower.copy()
    best_score = (float(np.linalg.norm(matrix @ best - target)), float(np.linalg.norm(best)))
    for assignment in product(*states):
        fixed = [i for i, status in enumerate(assignment) if status != 0]
        free = [i for i, status in enumerate(assignment) if status == 0]
        candidate = np.empty(count)
        if fixed:
            candidate[fixed] = [lower[i] if assignment[i] == 1 else upper[i] for i in fixed]
        rhs = target - (matrix[:, fixed] @ candidate[fixed] if fixed else 0.0)
        if free:
            values, *_ = np.linalg.lstsq(matrix[:, free], rhs, rcond=None)
            candidate[free] = values
            if np.any(values < lower[free] - 1e-10) or np.any(values > upper[free] + 1e-10):
                continue
        score = (float(np.linalg.norm(matrix @ candidate - target)), float(np.linalg.norm(candidate)))
        if score < best_score:
            best, best_score = candidate.copy(), score
    return np.clip(best, lower, upper)


def _projected_least_squares(matrix: NDArray[np.float64], target: NDArray[np.float64], lower: NDArray[np.float64], upper: NDArray[np.float64]) -> NDArray[np.float64]:
    value = np.clip(np.linalg.lstsq(matrix, target, rcond=None)[0], lower, upper)
    spectral = float(np.linalg.norm(matrix, 2))
    step = 1.0 / max(spectral * spectral, 1e-12)
    for _ in range(10_000):
        updated = np.clip(value - step * matrix.T @ (matrix @ value - target), lower, upper)
        if np.linalg.norm(updated - value) <= 1e-12 * (1.0 + np.linalg.norm(value)):
            return updated
        value = updated
    return value


def _separates_target(matrix, target, lower, upper, residual, threshold) -> bool:
    """Conservatively certify distance from the force set using its support.

    A small iterate change is not an optimality or infeasibility certificate.
    For unit w, w @ target - support(B [lower, upper], w) is a lower
    bound on the distance to that set. Floating-point cancellation can make
    this inconclusive; it must never be resolved by guessing infeasibility.
    """
    norm = float(np.linalg.norm(residual))
    if norm == 0.0 or not np.isfinite(norm):
        return False
    direction = -residual / norm
    with np.errstate(over="ignore", invalid="ignore"):
        coefficients = matrix.T @ direction
        # Overestimate each projection's support contribution, including
        # cancellation within B.T @ direction. Bounds are nonnegative, so
        # increasing a coefficient cannot decrease the support function.
        coefficient_error = (8 * np.finfo(float).eps * max(1, matrix.shape[0])
                             * (np.abs(matrix).T @ np.abs(direction)))
        coefficients = coefficients + coefficient_error
        positive = coefficients > 0.0
        if np.any(~np.isfinite(coefficients)) or np.any(~np.isfinite(upper[positive])):
            return False
        contributions = coefficients * lower
        contributions[positive] = coefficients[positive] * upper[positive]
        projected_target = float(direction @ target)
        support = float(np.sum(contributions))
        roundoff = (64 * np.finfo(float).eps * max(1, *matrix.shape)
                    * (np.sum(np.abs(direction * target)) + np.sum(np.abs(contributions))))
        gap = projected_target - support
    return bool(np.isfinite(gap) and np.isfinite(roundoff) and gap > threshold + roundoff)


def allocate_tensions(
    force_matrix: ArrayLike,
    target_generalized_force: ArrayLike,
    tension_min: ArrayLike,
    tension_max: ArrayLike,
    *,
    tolerance: float = 1e-8,
) -> TensionAllocationResult:
    matrix = np.asarray(force_matrix, dtype=float)
    target = np.asarray(target_generalized_force, dtype=float)
    lower = np.asarray(tension_min, dtype=float)
    upper = np.asarray(tension_max, dtype=float)
    if matrix.ndim != 2 or target.shape != (matrix.shape[0],):
        raise ValueError("force matrix and target shapes are incompatible")
    if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape:
        raise ValueError("tension bounds have the wrong shape")
    if np.any(lower < 0.0) or np.any(upper < lower):
        raise ValueError("invalid tension bounds")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target)) or not np.all(np.isfinite(lower)) or np.any(np.isnan(upper)):
        raise ValueError("matrix, target and lower bounds must be finite; upper bounds may be +inf")
    if not np.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be positive and finite")
    try:
        if matrix.size == 0 or not np.any(matrix):
            tensions = lower.copy()
        elif matrix.shape[1] <= 8:
            tensions = _enumerated_box_least_squares(matrix, target, lower, upper)
        else:
            tensions = _projected_least_squares(matrix, target, lower, upper)
    except np.linalg.LinAlgError as error:
        achieved = matrix @ lower
        return TensionAllocationResult(AllocationStatus.NUMERICAL_FAILURE, lower.copy(), achieved, target, achieved - target, float("inf"), str(error))
    with np.errstate(over="ignore", invalid="ignore"):
        achieved = matrix @ tensions
        residual = achieved - target
        norm = float(np.linalg.norm(residual))
        scale = 1.0 + float(np.linalg.norm(target))
    if not np.all(np.isfinite(achieved)) or not np.isfinite(norm) or not np.isfinite(scale):
        return TensionAllocationResult(AllocationStatus.NUMERICAL_FAILURE, tensions, achieved, target, residual, norm, "nonfinite force or residual arithmetic")
    feasible = norm <= tolerance * scale
    certified = not feasible and _separates_target(matrix, target, lower, upper, residual, tolerance * scale)
    status = AllocationStatus.FEASIBLE if feasible else (AllocationStatus.INFEASIBLE if certified else AllocationStatus.NUMERICALLY_UNRESOLVED)
    message = "bounded equilibrium found" if feasible else ("separating hyperplane certifies target outside the bounded cable-force set" if certified else "reference solver stopped without a feasibility or infeasibility certificate")
    return TensionAllocationResult(status, tensions, achieved, target, residual, norm, message)


def solve_tension_allocation(
    robot: "CableRobot",
    state: RobotState,
    external_load: ArrayLike | None = None,
    *,
    include_gravity: bool = True,
    tolerance: float = 1e-8,
) -> TensionAllocationResult:
    from cablerobot.statics.gravity import gravity_generalized_force

    applied = np.zeros(robot.dof)
    if include_gravity:
        applied += gravity_generalized_force(robot, state)
    if external_load is not None:
        extra = np.asarray(external_load, dtype=float)
        if extra.shape != (robot.dof,) or not np.all(np.isfinite(extra)):
            raise ValueError("external_load must be a finite generalized vector")
        applied += extra
    lower, upper = robot.tension_bounds()
    return allocate_tensions(robot.cable_force_matrix(state), -applied, lower, upper, tolerance=tolerance)
