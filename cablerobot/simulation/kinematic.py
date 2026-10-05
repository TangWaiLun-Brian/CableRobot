"""Sample prescribed generalized-coordinate motion without a dynamics claim."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from cablerobot.model.state import RobotState


@dataclass(frozen=True, slots=True)
class KinematicTrajectory:
    times: NDArray[np.float64]
    q: NDArray[np.float64]
    qd: NDArray[np.float64]
    fps: float

    def __post_init__(self) -> None:
        times = np.asarray(self.times, dtype=float).copy()
        q = np.asarray(self.q, dtype=float).copy()
        qd = np.asarray(self.qd, dtype=float).copy()
        if not np.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError("fps must be positive and finite")
        if times.ndim != 1 or times.size < 2:
            raise ValueError("a trajectory requires at least two time samples")
        if q.ndim != 2 or q.shape[0] != times.size or qd.shape != q.shape:
            raise ValueError("q and qd must have shape (sample_count, dof)")
        if not all(np.all(np.isfinite(array)) for array in (times, q, qd)):
            raise ValueError("trajectory samples must be finite")
        if not np.isclose(times[0], 0.0) or not np.allclose(np.diff(times), 1.0 / self.fps):
            raise ValueError("times must start at zero and advance uniformly by 1/fps")
        for name, value in (("times", times), ("q", q), ("qd", qd)):
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def duration(self) -> float:
        return len(self.times) / self.fps

    def state_at(self, index: int) -> RobotState:
        return RobotState(self.q[index].copy(), self.qd[index].copy())


def sinusoidal_trajectory(robot, initial_state: RobotState, amplitudes: ArrayLike, *,
                          duration: float = 5.0, fps: float = 20.0,
                          phases: ArrayLike | None = None) -> KinematicTrajectory:
    """One smooth periodic cycle with analytic qd and the given initial q.

    q(t) = q0 + A * (sin(2*pi*t/duration + phase) - sin(phase)).
    The last sample is at duration - 1/fps, avoiding a duplicated loop endpoint.
    Configuration values are prescribed; cable actuation/dynamics are not solved.
    """
    robot.validate()
    robot.validate_state(initial_state)
    amplitude = np.asarray(amplitudes, dtype=float)
    phase = np.zeros(robot.dof) if phases is None else np.asarray(phases, dtype=float)
    if amplitude.shape != (robot.dof,) or phase.shape != (robot.dof,):
        raise ValueError("amplitudes and phases must have shape (robot.dof,)")
    if not np.all(np.isfinite(amplitude)) or not np.all(np.isfinite(phase)):
        raise ValueError("amplitudes and phases must be finite")
    if not np.isfinite(duration) or duration <= 0 or not np.isfinite(fps) or fps <= 0:
        raise ValueError("duration and fps must be positive and finite")
    frame_count = int(round(duration * fps))
    if frame_count < 2 or not np.isclose(frame_count, duration * fps):
        raise ValueError("duration * fps must be an integer of at least two frames")
    times = np.arange(frame_count, dtype=float) / fps
    omega = 2 * np.pi / duration
    angle = omega * times[:, None] + phase
    q = initial_state.q + amplitude * (np.sin(angle) - np.sin(phase))
    qd = amplitude * omega * np.cos(angle)
    return KinematicTrajectory(times, q, qd, float(fps))
