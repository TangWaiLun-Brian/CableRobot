"""Small dependency-free nonlinear kinematic solvers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from cablerobot.kinematics.jacobians import numerical_jacobian
from cablerobot.model.state import RobotState

if False:  # pragma: no cover
    from cablerobot.model.robot import CableRobot


class SolverStatus(str, Enum):
    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
    NUMERICAL_FAILURE = "numerical_failure"


@dataclass(frozen=True, slots=True)
class KinematicSolverResult:
    status: SolverStatus
    state: RobotState
    residual_norm: float
    iterations: int
    message: str

    @property
    def converged(self) -> bool:
        return self.status is SolverStatus.CONVERGED


def _gauss_newton(
    residual: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    q0: ArrayLike,
    *,
    jacobian: Callable[[NDArray[np.float64]], NDArray[np.float64]] | None = None,
    tolerance: float = 1e-9,
    max_iterations: int = 100,
    damping: float = 1e-8,
) -> KinematicSolverResult:
    q = np.asarray(q0, dtype=float).copy()
    if tolerance <= 0.0 or max_iterations < 0 or damping <= 0.0:
        raise ValueError("solver tolerance/damping must be positive and iterations nonnegative")
    for iteration in range(max_iterations + 1):
        r = residual(q)
        norm = float(np.linalg.norm(r))
        if not np.isfinite(norm):
            return KinematicSolverResult(SolverStatus.NUMERICAL_FAILURE, RobotState(q), norm, iteration, "nonfinite residual")
        if norm <= tolerance:
            return KinematicSolverResult(SolverStatus.CONVERGED, RobotState(q), norm, iteration, "residual tolerance reached")
        if iteration == max_iterations:
            break
        J = jacobian(q) if jacobian is not None else numerical_jacobian(residual, q, 1e-6)
        try:
            lhs = J.T @ J + damping * np.eye(q.size)
            step = np.linalg.solve(lhs, -J.T @ r)
        except np.linalg.LinAlgError:
            return KinematicSolverResult(SolverStatus.NUMERICAL_FAILURE, RobotState(q), norm, iteration, "linear solve failed")
        # Backtracking makes the reference solver useful beyond a tiny local step.
        scale = 1.0
        for _ in range(20):
            candidate = q + scale * step
            if np.linalg.norm(residual(candidate)) < norm:
                q = candidate
                break
            scale *= 0.5
    return KinematicSolverResult(SolverStatus.MAX_ITERATIONS, RobotState(q), float(np.linalg.norm(residual(q))), max_iterations, "maximum iterations reached")


def solve_configuration_from_lengths(
    robot: "CableRobot",
    measured_lengths: ArrayLike,
    initial_state: RobotState,
    **solver_options: object,
) -> KinematicSolverResult:
    target = np.asarray(measured_lengths, dtype=float)
    robot.validate_state(initial_state)
    if target.shape != (robot.cable_count,):
        raise ValueError("measured_lengths has the wrong shape")
    if not np.all(np.isfinite(target)) or np.any(target < 0.0):
        raise ValueError("measured lengths must be finite and nonnegative")
    return _gauss_newton(
        lambda q: robot.cable_lengths(RobotState(q)) - target,
        initial_state.q,
        jacobian=lambda q: robot.cable_jacobian(RobotState(q)),
        **solver_options,
    )


def solve_frame_position(
    robot: "CableRobot",
    frame: str,
    desired_position: ArrayLike,
    initial_state: RobotState,
    **solver_options: object,
) -> KinematicSolverResult:
    from cablerobot.kinematics.bodies import frame_position

    target = np.asarray(desired_position, dtype=float)
    robot.validate_state(initial_state)
    if target.shape != (3,):
        raise ValueError("desired_position must have shape (3,)")
    return _gauss_newton(lambda q: frame_position(robot, frame, RobotState(q)) - target, initial_state.q, **solver_options)


def solve_frame_pose(robot: "CableRobot", frame: str, desired_transform: ArrayLike,
                     initial_state: RobotState, *, rotation_weight: float = 1.0,
                     **solver_options: object) -> KinematicSolverResult:
    """Solve an arbitrary selected frame pose locally, using a weighted SO(3) residual."""
    from cablerobot.model.transforms import rotvec_from_rotation, validate_transform

    target = validate_transform(desired_transform, "desired_transform")
    robot.validate_state(initial_state)
    if not np.isfinite(rotation_weight) or rotation_weight <= 0.0:
        raise ValueError("rotation_weight must be positive and finite")

    def residual(q):
        T = robot.frame_transform(frame, RobotState(q))
        return np.concatenate([T[:3, 3] - target[:3, 3], rotation_weight * rotvec_from_rotation(target[:3, :3].T @ T[:3, :3])])

    return _gauss_newton(residual, initial_state.q, **solver_options)
