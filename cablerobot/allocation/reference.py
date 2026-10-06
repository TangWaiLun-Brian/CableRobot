"""Bounded reference-tension QP using the common generalized-force mapping.

The foundation allocator remains an independent bounded-equilibrium baseline.
This module owns numerical arrays only; it has no hardware or topology assumptions.
See docs/tension_allocation.md for the dual derivation and certification limits.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .tension import AllocationStatus, TensionAllocationResult, allocate_tensions, _separates_target

if TYPE_CHECKING:
    from cablerobot.model import CableRobot, RobotState


@dataclass(frozen=True, slots=True)
class EquilibriumTensionResult(TensionAllocationResult):
    reference_tension: NDArray[np.float64]
    weights: NDArray[np.float64]
    objective_value: float | None
    active_lower: NDArray[np.bool_]
    active_upper: NDArray[np.bool_]
    lower_margin: NDArray[np.float64]
    upper_margin: NDArray[np.float64]
    equilibrium_feasible: bool
    optimality_verified: bool
    duality_gap: float | None
    dual_multiplier: NDArray[np.float64] | None
    iterations: int
    backend: str = "numpy-dual-newton"

    @property
    def tension_command(self) -> NDArray[np.float64] | None:
        """Only numerically verified equilibria expose a proposed command.

        Even this is model output, not hardware safety authorization.
        """
        return self.tensions.copy() if self.feasible else None


def _cable_vector(value: ArrayLike, count: int, name: str, *, positive=False):
    values = np.asarray(value, dtype=float)
    if values.ndim == 0:
        values = np.full(count, float(values))
        # Validate scalars even for a zero-cable problem.
        scalar = float(np.asarray(value))
        if not np.isfinite(scalar) or (scalar <= 0 if positive else scalar < 0):
            raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be a finite scalar or cable-count vector")
    if np.any(values <= 0 if positive else values < 0):
        raise ValueError(f"{name} must be {'positive' if positive else 'nonnegative'}")
    return values.copy()


def _make_result(status, matrix, target, lower, upper, reference, weights,
                 tensions, tolerance, iterations, message, multiplier=None):
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        achieved = matrix @ tensions
        residual = achieved - target
        residual_norm = float(np.linalg.norm(residual))
        threshold = tolerance * (1.0 + float(np.linalg.norm(target)))
        weighted_difference = weights * (tensions - reference)
        objective = float(0.5 * (weighted_difference @ weighted_difference))
        gap = None if multiplier is None else float(abs(multiplier @ residual))
    finite = (np.all(np.isfinite(tensions)) and np.all(np.isfinite(achieved))
              and np.isfinite(residual_norm) and np.isfinite(threshold)
              and np.isfinite(objective) and (gap is None or np.isfinite(gap)))
    bounded = bool(np.all(tensions >= lower) and np.all(tensions <= upper))
    equilibrium = bool(finite and bounded and residual_norm <= threshold)
    optimal = bool(equilibrium and gap is not None and gap <= tolerance * (1.0 + objective))
    if not finite:
        status = AllocationStatus.NUMERICAL_FAILURE
        message = "nonfinite objective, dual or force arithmetic; " + message
    elif status is AllocationStatus.FEASIBLE and not optimal:
        status = AllocationStatus.NUMERICALLY_UNRESOLVED
        message = "final equilibrium/objective checks did not verify the candidate"
    activity_tolerance = tolerance * (1.0 + np.abs(tensions))
    return EquilibriumTensionResult(
        status=status, tensions=tensions.copy(), achieved_generalized_force=achieved,
        target_generalized_force=target.copy(), residual=residual,
        residual_norm=residual_norm, message=message, reference_tension=reference,
        weights=weights, objective_value=objective if np.isfinite(objective) else None,
        active_lower=tensions - lower <= activity_tolerance,
        active_upper=upper - tensions <= activity_tolerance,
        lower_margin=tensions - lower, upper_margin=upper - tensions,
        equilibrium_feasible=equilibrium, optimality_verified=optimal,
        duality_gap=gap, dual_multiplier=None if multiplier is None else multiplier.copy(),
        iterations=iterations,
    )


def _dual_candidate(matrix, reference, weights, lower, upper, multiplier):
    return np.clip(reference + (matrix.T @ multiplier) / weights**2, lower, upper)


def _line_search(matrix, target, reference, weights, lower, upper, multiplier, direction):
    """Minimize the convex dual along a descent direction by monotone derivative.

    Avoid cancellation-prone comparisons of large dual objective values. No
    derivative root in the bounded search budget means unresolved, not infeasible.
    """
    def derivative(step):
        trial = multiplier + step * direction
        tensions = _dual_candidate(matrix, reference, weights, lower, upper, trial)
        return float(direction @ (matrix @ tensions - target))

    start, end = 0.0, 1.0
    for _ in range(80):
        value = derivative(end)
        if value >= 0:
            break
        start, end = end, end * 2.0
    else:
        return None
    if value == 0.0:
        return multiplier + end * direction
    best_step, best_value = end, abs(value)
    for _ in range(80):
        middle = 0.5 * (start + end)
        if middle == start or middle == end:
            break
        value = derivative(middle)
        if abs(value) < best_value:
            best_step, best_value = middle, abs(value)
        if value == 0.0:
            break
        if value > 0:
            end = middle
        else:
            start = middle
    updated = multiplier + best_step * direction
    return None if np.array_equal(updated, multiplier) else updated


def allocate_reference_tensions(
    force_matrix: ArrayLike,
    target_generalized_force: ArrayLike,
    tension_min: ArrayLike,
    tension_max: ArrayLike,
    *,
    reference_tension: ArrayLike = 0.0,
    weights: ArrayLike | None = None,
    tolerance: float = 1e-8,
    max_iterations: int = 200,
) -> EquilibriumTensionResult:
    """Minimize 0.5*||diag(weights)*(t-reference_tension)||^2 under B t=target.

    Scalar references and positive diagonal weights broadcast in cable order.
    Input errors raise ValueError; solver uncertainty is returned as a status.
    `feasible` requires both full equilibrium and numerical dual optimality checks.
    Other statuses may contain a bounded diagnostic candidate, never a command.
    """
    matrix = np.asarray(force_matrix, dtype=float)
    target = np.asarray(target_generalized_force, dtype=float)
    lower = np.asarray(tension_min, dtype=float)
    upper = np.asarray(tension_max, dtype=float)
    if matrix.ndim != 2 or target.shape != (matrix.shape[0],):
        raise ValueError("force matrix and target shapes are incompatible")
    if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape:
        raise ValueError("tension bounds have the wrong shape")
    if (not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(lower)) or np.any(np.isnan(upper))
            or np.any(lower < 0) or np.any(upper < lower)):
        raise ValueError("finite matrix/target/nonnegative lower bounds and upper >= lower required")
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("tolerance must be positive and finite")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, (int, np.integer)) or max_iterations < 1:
        raise ValueError("max_iterations must be a positive integer")
    reference = _cable_vector(reference_tension, matrix.shape[1], "reference_tension")
    weight = _cable_vector(1.0 if weights is None else weights, matrix.shape[1], "weights", positive=True)
    tensions = np.clip(reference, lower, upper)
    multiplier = np.zeros(matrix.shape[0])
    iteration = 0
    message = "dual iteration budget exhausted"
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="ignore"):
            scaled_matrix = matrix / weight
            for iteration in range(max_iterations + 1):
                tensions = _dual_candidate(matrix, reference, weight, lower, upper, multiplier)
                result = _make_result(AllocationStatus.FEASIBLE, matrix, target, lower, upper,
                                      reference, weight, tensions, tolerance, iteration,
                                      "bounded equilibrium and reference objective verified numerically", multiplier)
                if result.status in (AllocationStatus.FEASIBLE, AllocationStatus.NUMERICAL_FAILURE):
                    return result
                if iteration == max_iterations:
                    break
                # The dual Hessian is A_free A_free.T. SVD avoids forming/inverting
                # the normal matrix or eliminating any equilibrium coordinates.
                free = (tensions > lower) & (tensions < upper)
                direction = -result.residual
                if np.any(free):
                    u, singular, _ = np.linalg.svd(scaled_matrix[:, free], full_matrices=False)
                    if singular.size:
                        keep = singular > np.finfo(float).eps * max(scaled_matrix.shape) * singular[0]
                        newton = -u[:, keep] @ ((u[:, keep].T @ result.residual) / singular[keep]**2)
                        if (np.all(np.isfinite(newton)) and np.linalg.norm(newton) > 0
                                and -result.residual @ newton > 1e-6 * result.residual_norm * np.linalg.norm(newton)):
                            direction = newton
                magnitude = float(np.max(np.abs(direction), initial=0.0))
                if magnitude == 0.0:
                    message = "dual residual could not produce a descent direction"
                    break
                direction = direction / magnitude
                updated = _line_search(matrix, target, reference, weight, lower, upper, multiplier, direction)
                if updated is None:
                    message = "dual directional search stalled or exceeded its bracket budget"
                    break
                multiplier = updated
    except (np.linalg.LinAlgError, FloatingPointError) as error:
        return _make_result(AllocationStatus.NUMERICAL_FAILURE, matrix, target, lower, upper,
                            reference, weight, tensions, tolerance, iteration, str(error))

    # Reuse the conservative force-set separation from Milestone 1. An iteration
    # limit or lack of a dual root alone is NEVER evidence of infeasibility.
    candidate = _make_result(AllocationStatus.NUMERICALLY_UNRESOLVED, matrix, target, lower, upper,
                             reference, weight, tensions, tolerance, iteration, message, multiplier)
    threshold = tolerance * (1.0 + float(np.linalg.norm(target)))
    if _separates_target(matrix, target, lower, upper, candidate.residual, threshold):
        return _make_result(AllocationStatus.INFEASIBLE, matrix, target, lower, upper,
                            reference, weight, tensions, tolerance, iteration,
                            "separating hyperplane certifies no bounded equilibrium")
    baseline = allocate_tensions(matrix, target, lower, upper, tolerance=tolerance)
    # A fallback witness may improve feasibility but does not solve this QP.
    if baseline.residual_norm < candidate.residual_norm:
        candidate = _make_result(AllocationStatus.NUMERICALLY_UNRESOLVED, matrix, target, lower, upper,
                                 reference, weight, baseline.tensions, tolerance, iteration,
                                 message + "; foundation witness does not verify the reference optimum")
    if baseline.status in (AllocationStatus.INFEASIBLE, AllocationStatus.NUMERICAL_FAILURE):
        return _make_result(baseline.status, matrix, target, lower, upper, reference, weight,
                            candidate.tensions, tolerance, iteration, message + "; " + baseline.message)
    return candidate


def solve_equilibrium_tensions(
    robot: "CableRobot",
    state: "RobotState",
    external_load: ArrayLike | None = None,
    *,
    reference_tension: ArrayLike = 0.0,
    weights: ArrayLike | None = None,
    include_gravity: bool = True,
    tolerance: float = 1e-8,
    max_iterations: int = 200,
) -> EquilibriumTensionResult:
    """Allocate reference-optimal tensions for gravity + additional applied load.

    Reuses generic routes, B=-J_l.T and COM gravity. All generalized coordinates
    participate, including floating-platform orientation and serial joints.
    """
    from cablerobot.statics.gravity import gravity_generalized_force

    robot.validate_state(state)
    robot.validate()
    applied = gravity_generalized_force(robot, state) if include_gravity else np.zeros(robot.dof)
    if external_load is not None:
        extra = np.asarray(external_load, dtype=float)
        if extra.shape != (robot.dof,) or not np.all(np.isfinite(extra)):
            raise ValueError("external_load must be a finite generalized vector")
        applied = applied + extra
    lower, upper = robot.tension_bounds()
    return allocate_reference_tensions(robot.cable_force_matrix(state), -applied, lower, upper,
                                       reference_tension=reference_tension, weights=weights,
                                       tolerance=tolerance, max_iterations=max_iterations)
