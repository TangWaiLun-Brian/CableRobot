"""Matplotlib rendering of generic topology and routed cable segments."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from cablerobot.kinematics.bodies import body_transforms
from cablerobot.kinematics.cables import cable_route_points


def plot_robot(robot, state, *, ax=None, selected_frames: Sequence[str] | None = None,
               tensions=None, labels: bool = True, frame_scale: float = 0.15):
    """Draw body origins, tree links, frames, route points and cables; return Axes3D."""
    import matplotlib.pyplot as plt

    if ax is None:
        ax = plt.figure(figsize=(10, 7)).add_subplot(111, projection="3d")
    transforms = body_transforms(robot, state)
    points = []
    for body_index, (name, body) in enumerate(robot.bodies.items()):
        p = transforms[name][:3, 3]
        points.append(p)
        ax.scatter(*p, marker="s" if body.fixed else "o", s=max(30, 200 * body.visual_size), color="black" if body.fixed else "tab:blue")
        if labels:
            label_position = p + np.array([0.0, 0.0, 0.045 * (body_index + 1)])
            ax.text(*label_position, name, fontsize=8)
    for joint in robot.joints:
        p = transforms[joint.parent][:3, 3]
        c = transforms[joint.child][:3, 3]
        joint_world = transforms[joint.parent] @ joint.T_parent_joint
        j = joint_world[:3, 3]
        ax.plot(*np.vstack([p, j, c]).T, color="0.5", linewidth=3)
        ax.scatter(*j, marker="D", color="tab:orange", s=20)
        points.append(j)
    frames = list(robot.frames) if selected_frames is None else list(selected_frames)
    for name in frames:
        T = robot.frame_transform(name, state)
        origin = T[:3, 3]
        points.append(origin)
        for axis, color in enumerate(("r", "g", "b")):
            endpoint = origin + frame_scale * T[:3, axis]
            ax.plot(*np.vstack([origin, endpoint]).T, color=color, linewidth=1)
        if labels and name not in robot.bodies:
            ax.text(*origin, name, fontsize=7, color="0.4")
    if tensions is not None:
        tensions = np.asarray(tensions, dtype=float)
        if tensions.shape != (robot.cable_count,):
            raise ValueError("tensions has the wrong shape")
    for index, cable in enumerate(robot.cables):
        route = cable_route_points(robot, cable, state)
        points.extend(route)
        text = cable.name if tensions is None else f"{cable.name}: {tensions[index]:.2f} N"
        ax.plot(*route.T, color=f"C{index % 10}", linewidth=1.5, marker=".", label=text if labels else None)
    if points:
        coordinates = np.vstack(points)
        center = (coordinates.max(axis=0) + coordinates.min(axis=0)) / 2
        radius = max(float(np.ptp(coordinates, axis=0).max()) / 2, 0.5) * 1.15
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))
    if labels and robot.cables:
        ax.legend(loc="upper left", bbox_to_anchor=(1.04, 1.0), fontsize=8)
    ax.zaxis.labelpad = 0
    ax.set(xlabel="world x [m]", ylabel="world y [m]", zlabel="world z [m]", title=robot.name)
    return ax
