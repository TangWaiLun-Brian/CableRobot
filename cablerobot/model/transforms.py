"""Rigid-transform helpers using the T_A_B convention."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


def transform(rotation: ArrayLike | None = None, translation: ArrayLike | None = None) -> NDArray[np.float64]:
    """Return a homogeneous transform, defaulting to identity."""
    T = np.eye(4, dtype=float)
    if rotation is not None:
        R = np.asarray(rotation, dtype=float)
        if R.shape != (3, 3):
            raise ValueError("rotation must have shape (3, 3)")
        T[:3, :3] = R
    if translation is not None:
        p = np.asarray(translation, dtype=float)
        if p.shape != (3,):
            raise ValueError("translation must have shape (3,)")
        T[:3, 3] = p
    return T


def validate_transform(value: ArrayLike, name: str = "transform") -> NDArray[np.float64]:
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must have shape (4, 4)")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} must be finite")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError(f"{name} has an invalid homogeneous bottom row")
    if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-9):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-9):
        raise ValueError(f"{name} rotation must be proper (determinant +1)")
    return T.copy()


def skew(v: ArrayLike) -> NDArray[np.float64]:
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_from_rotvec(rotvec: ArrayLike) -> NDArray[np.float64]:
    """Rodrigues exponential map for a 3-vector rotation vector."""
    r = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        K = skew(r)
        return np.eye(3) + K + 0.5 * K @ K
    K = skew(r / theta)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * K @ K


def rotation_about_axis(axis: ArrayLike, angle: float) -> NDArray[np.float64]:
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm == 0.0:
        raise ValueError("joint axis must be nonzero")
    return rotation_from_rotvec(a / norm * angle)


def transform_point(T_a_b: ArrayLike, point_b: ArrayLike) -> NDArray[np.float64]:
    T = np.asarray(T_a_b, dtype=float)
    p = np.asarray(point_b, dtype=float)
    if p.shape != (3,):
        raise ValueError("point must have shape (3,)")
    return T[:3, :3] @ p + T[:3, 3]


def inverse_transform(T_a_b: ArrayLike) -> NDArray[np.float64]:
    T = np.asarray(T_a_b, dtype=float)
    R = T[:3, :3]
    p = T[:3, 3]
    return transform(R.T, -R.T @ p)


def rotvec_from_rotation(rotation: ArrayLike) -> NDArray[np.float64]:
    """Principal SO(3) logarithm, including the near-pi case."""
    R = np.asarray(rotation, dtype=float)
    angle = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    vee = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-8:
        return 0.5 * vee
    if np.pi - angle < 1e-6:
        values, vectors = np.linalg.eigh((R + R.T) / 2.0)
        axis = vectors[:, np.argmax(values)]
        if np.dot(axis, vee) < 0.0:
            axis = -axis
        return angle * axis
    return angle / (2.0 * np.sin(angle)) * vee
