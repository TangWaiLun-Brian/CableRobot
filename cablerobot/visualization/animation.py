"""Animations using the same body, frame and routed-cable geometry as statics."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from cablerobot.kinematics.cables import cable_route_points
from cablerobot.model import CableRobot
from cablerobot.simulation import KinematicTrajectory


EXTERNAL_COLOR = "#078ac6"
ONBOARD_COLOR = "#e98917"
BODY_COLOR = "#293d50"


def wireframe_box(size, center=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Local-coordinate box edges, shape (12, 2, 3), for rendering only."""
    size = np.asarray(size, dtype=float)
    center = np.asarray(center, dtype=float)
    if size.shape != (3,) or center.shape != (3,) or np.any(size <= 0):
        raise ValueError("box size must be a positive 3-vector and center a 3-vector")
    if not np.all(np.isfinite(size)) or not np.all(np.isfinite(center)):
        raise ValueError("box size and center must be finite")
    corners = np.array(list(product((-1., 1.), repeat=3)))
    edges = [(a, b) for a in range(8) for b in range(a + 1, 8)
             if np.count_nonzero(corners[a] != corners[b]) == 1]
    return np.array([[center + corners[a] * size / 2, center + corners[b] * size / 2]
                     for a, b in edges])


@dataclass(frozen=True)
class AnimationPanel:
    robot: CableRobot
    trajectory: KinematicTrajectory
    title: str = ""
    body_geometry: Mapping[str, np.ndarray] = field(default_factory=dict)
    selected_frames: tuple[str, ...] | None = None


@dataclass
class RobotAnimation:
    """A figure, its Matplotlib animation, and inspectable world-space cable artists."""

    figure: object
    animation: object
    panels: tuple[AnimationPanel, ...]
    cable_artists: tuple[dict, ...]
    draw_frame: object

    @property
    def frame_count(self) -> int:
        return len(self.panels[0].trajectory.times)

    @property
    def duration(self) -> float:
        return self.panels[0].trajectory.duration

    def save(self, path: str | Path, *, dpi: int = 100) -> None:
        from matplotlib.animation import FFMpegWriter, PillowWriter

        output = Path(path)
        fps = self.panels[0].trajectory.fps
        if output.suffix.lower() == ".gif":
            writer = PillowWriter(fps=fps)
        elif output.suffix.lower() == ".mp4":
            if not FFMpegWriter.isAvailable():
                raise RuntimeError("MP4 export requires FFmpeg; GIF export uses Matplotlib's Pillow writer")
            writer = FFMpegWriter(fps=fps, codec="libx264",
                                  extra_args=["-pix_fmt", "yuv420p", "-crf", "20"])
        else:
            raise ValueError("animation output must have .gif or .mp4 extension")
        output.parent.mkdir(parents=True, exist_ok=True)
        self.animation.save(str(output), writer=writer, dpi=dpi)

    def close(self) -> None:
        import matplotlib.pyplot as plt

        plt.close(self.figure)


class _RobotView:
    def __init__(self, ax, panel: AnimationPanel):
        self.ax, self.panel = ax, panel
        robot, trajectory = panel.robot, panel.trajectory
        robot.validate()
        if trajectory.q.shape[1] != robot.dof:
            raise ValueError("trajectory coordinate count does not match robot.dof")
        if any(name not in robot.bodies for name in panel.body_geometry):
            raise ValueError("body_geometry references an unknown body")
        self.geometry = {}
        for name, body in robot.bodies.items():
            edges = np.asarray(panel.body_geometry.get(name, wireframe_box([body.visual_size] * 3)), dtype=float)
            if edges.ndim != 3 or edges.shape[1:] != (2, 3) or not np.all(np.isfinite(edges)):
                raise ValueError("body geometry must contain finite edges of shape (edge_count, 2, 3)")
            self.geometry[name] = edges
        self.frames = tuple(robot.analysis_frames or robot.bodies) if panel.selected_frames is None else panel.selected_frames
        if any(name not in robot.frames for name in self.frames):
            raise ValueError("selected_frames references an unknown frame")
        self.external = np.array([
            any(robot.bodies[robot.frames[p.frame].body].fixed for p in cable.route.points)
            for cable in robot.cables
        ], dtype=bool)
        self.snapshots = [self.snapshot(trajectory.state_at(index)) for index in range(len(trajectory.times))]
        coordinates = np.vstack([
            points for snapshot in self.snapshots
            for points in [*snapshot["routes"], *snapshot["body_edges"].values(), snapshot["origins"]]
            if points.size
        ])
        lower, upper = coordinates.min(axis=0), coordinates.max(axis=0)
        span = np.maximum(upper - lower, 0.6)
        center = (lower + upper) / 2
        for setter, c, width in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center, span):
            setter(c - 0.60 * width, c + 0.60 * width)
        ax.set_box_aspect(span)
        ax.view_init(elev=23, azim=-58)
        ax.set_facecolor("#f7f9fc")
        ax.set(title=panel.title or robot.name, xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        ax.tick_params(labelsize=7, pad=1)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.label.set_fontsize(8)
            axis.labelpad = 1
        self.body_lines = {}
        for name, edges in self.geometry.items():
            fixed = robot.bodies[name].fixed
            self.body_lines[name] = [
                ax.plot([], [], [], color="#8b99a6" if fixed else BODY_COLOR,
                        linewidth=1.1 if fixed else (5.0 if len(edges) == 1 else 2.0),
                        alpha=0.65 if fixed else 1.0)[0]
                for _ in edges
            ]
        self.cables = {
            cable.name: ax.plot([], [], [], color=EXTERNAL_COLOR if self.external[index] else ONBOARD_COLOR,
                                linewidth=1.7 if self.external[index] else 2.3,
                                marker="o", markersize=3.0)[0]
            for index, cable in enumerate(robot.cables)
        }
        self.origins = ax.plot([], [], [], linestyle="", marker="o", color=BODY_COLOR, markersize=4)[0]
        self.frame_lines = [
            ax.plot([], [], [], color=color, alpha=0.7, linewidth=1.0)[0]
            for _ in self.frames for color in ("#c44848", "#409667", "#4779b5")
        ]
        self.length_text = ax.text2D(0.04, -0.14, "", transform=ax.transAxes, fontsize=8, color=BODY_COLOR)

    def snapshot(self, state):
        robot = self.panel.robot
        transforms = robot.body_transforms(state)
        routes = [cable_route_points(robot, cable, state) for cable in robot.cables]
        for route in routes:
            if np.any(np.linalg.norm(np.diff(route, axis=0), axis=1) < 1e-10):
                raise ValueError("an animation trajectory contains a zero-length cable segment")
        body_edges = {
            name: (edges @ transforms[name][:3, :3].T + transforms[name][:3, 3]).reshape(-1, 3)
            for name, edges in self.geometry.items()
        }
        frame_edges = []
        for name in self.frames:
            spec = robot.frames[name]
            T = transforms[spec.body] @ spec.T_body_frame
            frame_edges.extend([np.vstack([T[:3, 3], T[:3, 3] + 0.10 * T[:3, axis]])
                                for axis in range(3)])
        return {
            "routes": routes,
            "lengths": np.array([np.linalg.norm(np.diff(route, axis=0), axis=1).sum() for route in routes]),
            "body_edges": body_edges,
            "origins": np.vstack([T[:3, 3] for T in transforms.values()]),
            "frame_edges": frame_edges,
        }

    def update(self, index):
        snapshot = self.snapshots[index]
        artists = []
        for name, lines in self.body_lines.items():
            for line, edge in zip(lines, snapshot["body_edges"][name].reshape(-1, 2, 3)):
                line.set_data_3d(*edge.T)
                artists.append(line)
        for cable, points in zip(self.panel.robot.cables, snapshot["routes"]):
            line = self.cables[cable.name]
            line.set_data_3d(*points.T)
            artists.append(line)
        for line, points in zip(self.frame_lines, snapshot["frame_edges"]):
            line.set_data_3d(*points.T)
            artists.append(line)
        self.origins.set_data_3d(*snapshot["origins"].T)
        summaries = []
        for mask, label in ((self.external, "Outer-frame"), (~self.external, "On-robot")):
            values = snapshot["lengths"][mask]
            if values.size:
                summaries.append(f"{label} cable lengths: {values.min():.2f}–{values.max():.2f} m")
        self.length_text.set_text("\n".join(summaries))
        return [*artists, self.origins, self.length_text]


def animate_robots(panels: Sequence[AnimationPanel], *, figsize=None) -> RobotAnimation:
    """Animate synchronized robots with fixed cameras and full routed polylines."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.lines import Line2D

    panels = tuple(panels)
    if not panels:
        raise ValueError("at least one animation panel is required")
    first = panels[0].trajectory
    if any(len(panel.trajectory.times) != len(first.times) or
           not np.isclose(panel.trajectory.fps, first.fps) for panel in panels):
        raise ValueError("animation panels must have the same frame count and fps")
    figure = plt.figure(figsize=figsize or ((9, 6.4) if len(panels) == 1 else (16, 6.4)), facecolor="white")
    views = [_RobotView(figure.add_subplot(1, len(panels), index + 1, projection="3d"), panel)
             for index, panel in enumerate(panels)]
    figure.subplots_adjust(left=0.025, right=0.96, bottom=0.17, top=0.81, wspace=0.07)
    figure.suptitle("Cable routing across robot topologies", fontsize=17, color=BODY_COLOR, y=0.97)
    caption = figure.text(0.5, 0.90, "", ha="center", fontsize=10, color="#526678")
    figure.legend(handles=[
        Line2D([0], [0], color=EXTERNAL_COLOR, linewidth=2, marker="o", markersize=4, label="Outer-frame → robot"),
        Line2D([0], [0], color=ONBOARD_COLOR, linewidth=2, marker="o", markersize=4, label="Entirely on robot"),
        Line2D([0], [0], color=BODY_COLOR, linewidth=4, label="Rigid bodies / links"),
    ], loc="lower center", ncol=3, frameon=False, fontsize=10, bbox_to_anchor=(0.5, 0.02))

    def draw_frame(index):
        artists = [artist for view in views for artist in view.update(index)]
        caption.set_text(f"Prescribed kinematics · t = {first.times[index]:.2f} / {first.duration:.2f} s · {first.fps:g} fps")
        return [*artists, caption]

    draw_frame(0)
    animation = FuncAnimation(figure, draw_frame, frames=len(first.times), init_func=lambda: draw_frame(0),
                              interval=1000 / first.fps, blit=False, repeat=True, cache_frame_data=False)
    return RobotAnimation(figure, animation, panels, tuple(view.cables for view in views), draw_frame)


def animate_robot(robot, trajectory: KinematicTrajectory, *, title="", body_geometry=None,
                  selected_frames=None, figsize=None) -> RobotAnimation:
    """Convenience entry point for one topology."""
    return animate_robots([AnimationPanel(robot, trajectory, title, body_geometry or {},
                                          selected_frames)], figsize=figsize)
