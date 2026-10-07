"""Hierarchical bounded residual/reference allocation, separate from exact APIs."""

from dataclasses import dataclass, replace
from enum import Enum

import numpy as np
from numpy.typing import NDArray

from .reference import _cable_vector, allocate_reference_tensions, solve_equilibrium_tensions
from .scaling import ResidualScaling, residual_scaling
from .tension import AllocationStatus, allocate_tensions, _separates_target


class BoundedAllocationStatus(str, Enum):
    FEASIBLE = "feasible"
    BEST_EFFORT = "best_effort"
    INFEASIBLE = "infeasible"  # exact-only request, not a least-squares failure
    NUMERICALLY_UNRESOLVED = "numerically_unresolved"
    NUMERICAL_FAILURE = "numerical_failure"


@dataclass(frozen=True, slots=True)
class BoundedTensionResult:
    status: BoundedAllocationStatus
    tensions: NDArray[np.float64]
    achieved_generalized_force: NDArray[np.float64]
    target_generalized_force: NDArray[np.float64]
    residual: NDArray[np.float64]
    residual_norm: float
    weighted_residual_norm: float
    exact_equilibrium: bool
    objective_value: float | None
    reference_objective_value: float | None
    reference_tension: NDArray[np.float64]
    weights: NDArray[np.float64]
    reference_distance_norm: float
    residual_scaling: ResidualScaling
    force_residual: NDArray[np.float64] | None
    moment_residual: NDArray[np.float64] | None
    active_lower: NDArray[np.bool_]
    active_upper: NDArray[np.bool_]
    lower_margin: NDArray[np.float64]
    upper_margin: NDArray[np.float64]
    primary_optimality_verified: bool
    secondary_optimality_verified: bool
    primary_duality_gap: float | None
    primary_kkt_violation: float | None
    stage1_weighted_residual_norm: float | None
    force_image_change_norm: float | None
    exact_result: object | None
    secondary_result: object | None
    equilibrium_tolerance: float
    optimality_tolerance: float
    infeasibility_certified: bool
    message: str
    backend: str = "numpy-hierarchical-reference"

    @property
    def feasible(self):
        return self.status is BoundedAllocationStatus.FEASIBLE

    @property
    def equilibrium_feasible(self):
        return self.exact_equilibrium

    @property
    def tension_command(self):
        """Copied model proposal, NOT permission to drive hardware."""
        return self.tensions.copy() if self.status in (BoundedAllocationStatus.FEASIBLE,
                                                      BoundedAllocationStatus.BEST_EFFORT) else None


def _options(tolerance, optimality_tolerance, max_iterations):
    for value in (tolerance, optimality_tolerance):
        if not np.isfinite(value) or value <= 0:
            raise ValueError("equilibrium/optimality tolerances must be positive and finite")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, (int, np.integer)) or max_iterations < 1:
        raise ValueError("max_iterations must be a positive integer")


def _primary_certificate(matrix, target, lower, upper, tensions, tolerance):
    """Convex box KKT and guarded primal/dual gap, NOT the old status label.

    f(x)-min f <= g.T x-min_box g.T z. Infinite negative-gradient upper
    support is inconclusive even for a tiny coefficient: don't invent a cap.
    """
    residual = matrix @ tensions - target
    gradient = matrix.T @ residual
    objective = float(.5 * (residual @ residual))
    spectral = float(np.linalg.norm(matrix, 2)) if matrix.size else 0.
    # Projected stationarity is continuous at active bounds. Exact equality of
    # a computed tension to an upper bound would reject roundoff-sized interior
    # offsets with a legitimate nonzero bound multiplier. The separate guarded
    # dual gap still limits GLOBAL objective error, not just iterate movement.
    lipschitz = max(1., spectral*spectral)
    violation = lipschitz * (tensions-np.clip(tensions-gradient/lipschitz,lower,upper))
    kkt = float(np.max(np.abs(violation), initial=0.))
    kkt_limit = tolerance * (1 + spectral * np.linalg.norm(residual))
    negative, positive = gradient < 0, gradient > 0
    if np.any(~np.isfinite(upper[negative])):
        return False, None, kkt
    gap = float(gradient[positive] @ (tensions[positive] - lower[positive])
                - gradient[negative] @ (upper[negative] - tensions[negative]))
    # Guard cancellation in residual/gradient/support, with declared finite
    # ranges where support is used. No infinite*zero multiplication.
    ranges = np.zeros_like(tensions)
    finite = np.isfinite(upper)
    ranges[finite] = upper[finite] - lower[finite]
    ranges[~finite] = tensions[~finite] - lower[~finite]
    eps = 64 * np.finfo(float).eps * max(1, *matrix.shape)
    gradient_error = eps * (np.abs(matrix).T @ (np.abs(matrix) @ np.abs(tensions) + np.abs(target)))
    guarded_gap = max(0., gap) + float(gradient_error @ ranges) + eps * (1 + 2*objective)
    verified = (np.isfinite(guarded_gap) and np.isfinite(kkt) and kkt <= kkt_limit
                and guarded_gap <= tolerance * (1 + objective))
    return bool(verified), guarded_gap, kkt


def _result(status, matrix, target, lower, upper, reference, weights, scaling,
            tensions, tolerance, message, *, primary=False, secondary=False,
            gap=None, kkt=None, stage1=None, image_change=None, exact_result=None,
            achieved_override=None, secondary_result=None, optimality_tolerance=1e-8,
            certified=False):
    tensions = np.clip(np.where(np.isfinite(tensions), tensions, lower), lower, upper)
    with np.errstate(over="ignore", invalid="ignore"):
        achieved = matrix @ tensions if achieved_override is None else achieved_override.copy()
        residual = achieved - target
        norm = float(np.linalg.norm(residual))
        weighted = float(np.linalg.norm(scaling.matrix @ residual))
        distance = float(np.linalg.norm(tensions-reference))
        difference = weights*(tensions-reference)
        ref_objective = float(.5 * (difference @ difference))
        objective = float(.5 * np.square(np.float64(weighted)))
        threshold = tolerance * (1 + float(np.linalg.norm(target)))
        physical = None if scaling.physical_from_generalized is None else scaling.physical_from_generalized @ residual
    finite = np.all(np.isfinite(residual)) and all(np.isfinite(v) for v in (norm, weighted, distance, ref_objective, objective, threshold))
    if physical is not None:
        finite = finite and np.all(np.isfinite(physical))
    if not finite:
        status, primary, secondary = BoundedAllocationStatus.NUMERICAL_FAILURE, False, False
        message = "nonfinite objective/force/scaling arithmetic; " + message
    activity = tolerance * (1 + np.abs(tensions))
    return BoundedTensionResult(
        status, tensions.copy(), achieved, target.copy(), residual, norm, weighted,
        bool(finite and norm <= threshold), objective if np.isfinite(objective) else None,
        ref_objective if np.isfinite(ref_objective) else None, reference.copy(), weights.copy(),
        distance, scaling, None if physical is None else physical[:3].copy(),
        None if physical is None else physical[3:].copy(), tensions-lower <= activity,
        upper-tensions <= activity, tensions-lower, upper-tensions, primary, secondary,
        gap, kkt, stage1, image_change, exact_result, secondary_result,
        tolerance, optimality_tolerance, bool(certified and finite), message,
    )


def allocate_best_effort_tensions(force_matrix, target_generalized_force, tension_min,
                                 tension_max, *, residual_weights, reference_tension=0.,
                                 weights=None, tolerance=1e-8, optimality_tolerance=1e-8,
                                 max_iterations=200):
    """Minimize scaled residual first, then reference distance at the same force image.

    Explicit residual scaling is mandatory. Original exact APIs are untouched.
    A nonzero-residual BEST_EFFORT command cannot maintain the pose by itself.
    """
    _options(tolerance, optimality_tolerance, max_iterations)
    matrix, target = np.asarray(force_matrix, dtype=float), np.asarray(target_generalized_force, dtype=float)
    lower, upper = np.asarray(tension_min, dtype=float), np.asarray(tension_max, dtype=float)
    if matrix.ndim != 2 or target.shape != (matrix.shape[0],):
        raise ValueError("force matrix and target shapes are incompatible")
    scaling = residual_scaling(residual_weights, matrix.shape[0])
    reference = _cable_vector(reference_tension, matrix.shape[1], "reference_tension")
    weight = _cable_vector(1. if weights is None else weights, matrix.shape[1], "weights", positive=True)
    # Accepted allocator owns numerical box/problem validation. Validate BEFORE
    # scaling arithmetic, including errors that would otherwise appear as failure.
    if (lower.shape != (matrix.shape[1],) or upper.shape != lower.shape
            or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(lower)) or np.any(np.isnan(upper))
            or np.any(lower < 0) or np.any(upper < lower)):
        raise ValueError("finite matrix/target/nonnegative lower and upper >= lower required")
    tensions, gap, kkt, rho = lower.copy(), None, None, None
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="ignore"):
            a, b = scaling.matrix @ matrix, scaling.matrix @ target
            first = allocate_tensions(a, b, lower, upper, tolerance=optimality_tolerance)
            tensions = first.tensions
            if first.status is AllocationStatus.NUMERICAL_FAILURE:
                return _result(BoundedAllocationStatus.NUMERICAL_FAILURE, matrix, target, lower, upper,
                               reference, weight, scaling, tensions, tolerance, first.message,
                               optimality_tolerance=optimality_tolerance)
            verified, gap, kkt = _primary_certificate(a, b, lower, upper, tensions, optimality_tolerance)
            rho = float(np.linalg.norm(a @ tensions-b))
            if not verified:
                return _result(BoundedAllocationStatus.NUMERICALLY_UNRESOLVED, matrix, target, lower, upper,
                               reference, weight, scaling, tensions, tolerance,
                               "primary box optimality checks inconclusive", gap=gap, kkt=kkt, stage1=rho,
                               optimality_tolerance=optimality_tolerance)
            image = matrix @ tensions
            # Solve the existing reference QP on the unique optimal force image.
            # Tighten its equality tolerance to protect the separate primary budget;
            # never relax the accepted exact-equilibrium tolerance.
            secondary_tolerance = min(tolerance, optimality_tolerance /
                                      (100 * (1 + np.linalg.norm(image)) * max(1., np.linalg.norm(scaling.matrix, 2) if scaling.matrix.size else 0.)))
            if not np.isfinite(secondary_tolerance) or secondary_tolerance <= 0:
                raise FloatingPointError("secondary tolerance arithmetic underflow/overflow")
            second = allocate_reference_tensions(matrix, image, lower, upper, reference_tension=reference,
                                                 weights=weight, tolerance=secondary_tolerance,
                                                 max_iterations=max_iterations)
            if not second.feasible:
                status = (BoundedAllocationStatus.NUMERICAL_FAILURE if second.status is AllocationStatus.NUMERICAL_FAILURE
                          else BoundedAllocationStatus.NUMERICALLY_UNRESOLVED)
                return _result(status, matrix, target, lower, upper, reference, weight, scaling,
                               tensions, tolerance, "secondary reference selection not verified: " + second.message,
                               primary=True, gap=gap, kkt=kkt, stage1=rho, secondary_result=second,
                               optimality_tolerance=optimality_tolerance)
            change = float(np.linalg.norm(scaling.matrix @ (matrix @ second.tensions-image)))
            final_verified, final_gap, final_kkt = _primary_certificate(a, b, lower, upper,
                                                                       second.tensions, optimality_tolerance)
            preserved = change <= optimality_tolerance * (1 + rho)
            if not final_verified or not preserved:
                return _result(BoundedAllocationStatus.NUMERICALLY_UNRESOLVED, matrix, target, lower, upper,
                               reference, weight, scaling, tensions, tolerance, "secondary changed primary force image or optimality",
                               primary=True, gap=gap, kkt=kkt, stage1=rho, image_change=change,
                               secondary_result=second, optimality_tolerance=optimality_tolerance)
            tensions = second.tensions
            exact = np.linalg.norm(matrix @ tensions-target) <= tolerance*(1+np.linalg.norm(target))
            certified = not exact and _separates_target(
                matrix,target,lower,upper,scaling.matrix.T @ (a @ tensions-b),
                tolerance*(1+np.linalg.norm(target)))
            status = (BoundedAllocationStatus.FEASIBLE if exact else
                      BoundedAllocationStatus.BEST_EFFORT if certified else BoundedAllocationStatus.NUMERICALLY_UNRESOLVED)
            message = ("hierarchical residual and reference objectives verified numerically" if exact or certified else
                       "primary/reference checks passed but original-space infeasibility certificate is inconclusive")
            return _result(status, matrix, target, lower, upper, reference, weight, scaling, tensions,
                           tolerance, message,
                           primary=True, secondary=True, gap=final_gap, kkt=final_kkt, stage1=rho, image_change=change,
                           secondary_result=second, optimality_tolerance=optimality_tolerance, certified=certified)
    except (np.linalg.LinAlgError, FloatingPointError) as error:
        return _result(BoundedAllocationStatus.NUMERICAL_FAILURE, matrix, target, lower, upper, reference,
                       weight, scaling, tensions, tolerance, str(error), gap=gap, kkt=kkt, stage1=rho,
                       optimality_tolerance=optimality_tolerance)


def solve_bounded_equilibrium_tensions(robot, state, external_load=None, *, residual_weights,
                                       reference_tension=0., weights=None, include_gravity=True,
                                       best_effort=True, tolerance=1e-8, optimality_tolerance=1e-8,
                                       max_iterations=200):
    """Exact-first opt-in wrapper; useful approximation is NEVER labeled equilibrium."""
    _options(tolerance, optimality_tolerance, max_iterations)
    if not isinstance(best_effort, (bool, np.bool_)):
        raise ValueError("best_effort must be boolean")
    scaling = residual_scaling(residual_weights, robot.dof)
    exact = solve_equilibrium_tensions(robot, state, external_load, reference_tension=reference_tension,
                                       weights=weights, include_gravity=include_gravity,
                                       tolerance=tolerance, max_iterations=max_iterations)
    if exact.feasible or not best_effort or exact.status is AllocationStatus.NUMERICAL_FAILURE:
        # Recover achieved force without querying B again. Exact-only calls never
        # invoke the residual optimizer; all accepted tension/dual diagnostics remain.
        lower, upper = robot.tension_bounds()
        return _result(BoundedAllocationStatus(exact.status.value), None,
                         exact.target_generalized_force, lower, upper, exact.reference_tension, exact.weights,
                         scaling, exact.tensions, tolerance, exact.message, primary=exact.feasible,
                         secondary=exact.optimality_verified, exact_result=exact,
                         achieved_override=exact.achieved_generalized_force,
                         optimality_tolerance=optimality_tolerance,
                         certified=exact.status is AllocationStatus.INFEASIBLE)
    lower, upper = robot.tension_bounds()
    result = allocate_best_effort_tensions(robot.cable_force_matrix(state), exact.target_generalized_force,
                                         lower, upper, residual_weights=scaling,
                                         reference_tension=reference_tension, weights=weights,
                                         tolerance=tolerance, optimality_tolerance=optimality_tolerance,
                                         max_iterations=max_iterations)
    return replace(result, exact_result=exact)
