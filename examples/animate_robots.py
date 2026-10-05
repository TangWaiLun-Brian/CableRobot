"""Render spatial, serial and hybrid routing animations from common kinematics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from cablerobot.examples import routing_animation_panels
from cablerobot.io import save_robot
from cablerobot.visualization import animate_robots


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("examples/output/animations"))
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--format", choices=("gif", "mp4", "both"), default="gif")
    parser.add_argument("--robot", choices=("all", "spatial", "serial", "hybrid"), default="all")
    parser.add_argument("--ffmpeg", type=Path, help="Optional FFmpeg executable path for MP4 export")
    args = parser.parse_args()
    if args.ffmpeg:
        import matplotlib
        matplotlib.rcParams["animation.ffmpeg_path"] = str(args.ffmpeg)
    all_panels = routing_animation_panels(duration=args.duration, fps=args.fps)
    panels = all_panels if args.robot == "all" else {args.robot: all_panels[args.robot]}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    formats = ("gif", "mp4") if args.format == "both" else (args.format,)
    traces = {}
    for name, panel in panels.items():
        robot, trajectory = panel.robot, panel.trajectory
        lengths = np.vstack([robot.cable_lengths(trajectory.state_at(i))
                             for i in range(len(trajectory.times))])
        external = [
            any(robot.bodies[robot.frames[p.frame].body].fixed for p in cable.route.points)
            for cable in robot.cables
        ]
        traces[name] = {
            "robot_name": robot.name,
            "dof": robot.dof,
            "cable_count": robot.cable_count,
            "times_s": trajectory.times.tolist(),
            "q": trajectory.q.tolist(),
            "qd": trajectory.qd.tolist(),
            "cable_names": [c.name for c in robot.cables],
            "cable_routes": [[p.frame for p in c.route.points] for c in robot.cables],
            "routing_class": ["outer_frame_to_robot" if flag else "entirely_on_robot" for flag in external],
            "cable_lengths_m": lengths.tolist(),
            "length_range_m": np.ptp(lengths, axis=0).tolist(),
        }
        save_robot(robot, args.output_dir / f"{name}_model.json")
        rendered = animate_robots([panel])
        try:
            for extension in formats:
                path = args.output_dir / f"{name}.{extension}"
                rendered.save(path)
                print(f"Saved {path} ({rendered.frame_count} frames, {rendered.duration:g} s)", flush=True)
        finally:
            rendered.close()
    if len(panels) > 1:
        rendered = animate_robots(list(panels.values()))
        try:
            for extension in formats:
                path = args.output_dir / f"three_robots.{extension}"
                rendered.save(path)
                print(f"Saved {path} ({rendered.frame_count} frames, {rendered.duration:g} s)", flush=True)
        finally:
            rendered.close()
    report = {
        "motion": "prescribed_kinematics",
        "dynamics_solved": False,
        "route_model": "straight massless segments through body-fixed attachment/guide points",
        "duration_s": args.duration,
        "fps": args.fps,
        "robots": traces,
    }
    trace_path = args.output_dir / "motion_traces.json"
    trace_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Saved {trace_path}", flush=True)


if __name__ == "__main__":
    main()
