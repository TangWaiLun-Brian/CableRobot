"""Dynamics contracts for M(q) qdd + h(q, qd) = B(q)t + tau_ext."""

from __future__ import annotations

from typing import Protocol

import numpy as np


class DynamicsProvider(Protocol):
    """Numerical multibody provider; Python remains the model owner."""

    def mass_matrix(self, robot, state): ...

    def bias_force(self, robot, state): ...


def validate_generalized_vector(robot, state, value, name):
    robot.validate_state(state)
    robot.validate()
    vector = np.asarray(value, dtype=float)
    if vector.shape != (robot.dof,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite generalized vector of shape ({robot.dof},)")
    return vector


def numerical_terms(robot, state, provider: DynamicsProvider):
    matrix = np.asarray(provider.mass_matrix(robot, state), dtype=float)
    bias = validate_generalized_vector(robot, state, provider.bias_force(robot, state), "bias force")
    if matrix.shape != (robot.dof, robot.dof) or not np.all(np.isfinite(matrix)):
        raise ValueError("mass matrix has an invalid shape or nonfinite values")
    if not np.allclose(matrix, matrix.T, atol=1e-10, rtol=0):
        raise ValueError("mass matrix must be symmetric")
    if robot.dof:
        try:
            np.linalg.cholesky(matrix)
        except np.linalg.LinAlgError as error:
            raise ValueError("mass matrix must be positive definite") from error
    return matrix, bias


class FullDynamicsDeferredError(NotImplementedError):
    pass


def mass_matrix(robot, state):
    raise FullDynamicsDeferredError("full multibody mass matrices are deferred beyond milestone 1")


def bias_force(robot, state):
    raise FullDynamicsDeferredError("velocity-dependent multibody bias forces are deferred beyond milestone 1")
