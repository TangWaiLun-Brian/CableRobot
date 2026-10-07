"""Explicit generalized-residual scaling; no implicit mixed-unit objective."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class ResidualScaling:
    matrix: NDArray[np.float64]
    description: str = "explicit generalized-residual scaling"
    physical_from_generalized: NDArray[np.float64] | None = None

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=float).copy()
        if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1] or not np.all(np.isfinite(matrix)):
            raise ValueError("residual scaling must be a finite square matrix")
        if matrix.size:
            singular = np.linalg.svd(matrix, compute_uv=False)
            if singular[-1] <= 1e-12 * singular[0]:
                raise ValueError("residual scaling must be nonsingular and well conditioned")
        if not isinstance(self.description, str) or not self.description:
            raise ValueError("scaling description must be a nonempty string")
        physical = self.physical_from_generalized
        if physical is not None:
            physical = np.asarray(physical, dtype=float).copy()
            if physical.shape != (6, matrix.shape[0]) or not np.all(np.isfinite(physical)):
                raise ValueError("physical wrench map must be finite with shape (6, residual rows)")
            physical.setflags(write=False)
        matrix.setflags(write=False)
        object.__setattr__(self, "matrix", matrix)
        object.__setattr__(self, "physical_from_generalized", physical)


def residual_scaling(value, rows):
    """Validate explicit positive diagonal weights or a full ResidualScaling map."""
    if value is None:
        raise ValueError("residual_weights/scaling is required; no implicit mixed-unit default")
    if isinstance(value, ResidualScaling):
        if value.matrix.shape != (rows, rows):
            raise ValueError("residual scaling shape does not match generalized-force rows")
        return value
    weights = np.asarray(value, dtype=float)
    if weights.ndim == 0:
        if not np.isfinite(weights) or weights <= 0:
            raise ValueError("residual weights must be positive and finite")
        weights = np.full(rows, float(weights))
    if weights.shape != (rows,) or not np.all(np.isfinite(weights)) or np.any(weights <= 0):
        raise ValueError("residual weights must be a positive scalar or residual-row vector")
    return ResidualScaling(np.diag(weights), "explicit diagonal generalized-residual weights")


def spatial_wrench_scaling(characteristic_length, *, generalized_force_from_wrench=None):
    """Scale an explicit world [F; M] residual to force-equivalent units.

    Identity is ONLY for already Cartesian wrench input. For floating rotvec
    covectors pass the selected frame twist Jacobian transpose as the nonsingular
    wrench-to-generalized map. This helper does not infer topology/coordinates.
    """
    length = float(characteristic_length)
    if not np.isfinite(length) or length <= 0:
        raise ValueError("characteristic_length must be positive and finite")
    mapping = np.eye(6) if generalized_force_from_wrench is None else np.asarray(generalized_force_from_wrench, dtype=float)
    if mapping.shape != (6, 6) or not np.all(np.isfinite(mapping)):
        raise ValueError("wrench-to-generalized map must be a finite 6-by-6 matrix")
    singular = np.linalg.svd(mapping, compute_uv=False)
    if singular[-1] <= 1e-12 * singular[0]:
        raise ValueError("wrench-to-generalized map is singular or ill conditioned")
    physical = np.linalg.solve(mapping, np.eye(6))
    diagonal = np.diag([1., 1., 1., 1/length, 1/length, 1/length])
    return ResidualScaling(diagonal @ physical,
                           f"world Cartesian wrench, moment/characteristic_length={length:g} m",
                           physical)
