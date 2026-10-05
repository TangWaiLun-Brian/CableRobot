"""Versioned human-readable JSON model descriptions; Python owns semantics."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from cablerobot.model import Attachment, Body, Cable, CableRobot, CableRoute, Frame, Joint, RobotParameters


def robot_to_dict(robot: CableRobot) -> dict:
    robot.validate()
    for name in robot.bodies:
        if not np.allclose(robot.frames[name].T_body_frame, np.eye(4), atol=1e-12):
            raise ValueError("same-named body frames must remain identity; add a separate offset frame")
    return {
        "schema_version": 1,
        "name": robot.name,
        "parameters": {"gravity": robot.parameters.gravity.tolist(), "finite_difference_step": robot.parameters.finite_difference_step},
        "analysis_frames": list(robot.analysis_frames),
        "bodies": [{"name": b.name, "mass": b.mass, "center_of_mass": b.center_of_mass.tolist(), "inertia": b.inertia.tolist(), "fixed": b.fixed, "visual_size": b.visual_size} for b in robot.bodies.values()],
        "joints": [{"name": j.name, "parent": j.parent, "child": j.child, "joint_type": j.joint_type.value, "axis": j.axis.tolist(), "T_parent_joint": j.T_parent_joint.tolist(), "T_joint_child": j.T_joint_child.tolist()} for j in robot.joints],
        "frames": [{"name": f.name, "body": f.body, "T_body_frame": f.T_body_frame.tolist()} for f in robot.frames.values() if f.name not in robot.bodies],
        "cables": [{"name": c.name, "tension_min": c.tension_min, "tension_max": c.tension_max if np.isfinite(c.tension_max) else "inf", "stiffness": c.stiffness, "route": [{"frame": a.frame, "point": a.point.tolist(), "label": a.label} for a in c.route.points]} for c in robot.cables],
    }


def robot_from_dict(data: dict) -> CableRobot:
    if data.get("schema_version") != 1:
        raise ValueError("unsupported robot schema_version; expected 1")
    robot = CableRobot(data["name"], RobotParameters(**data.get("parameters", {})), tuple(data.get("analysis_frames", ())))
    for item in data["bodies"]:
        robot.add_body(Body(**item))
    for item in data["joints"]:
        robot.add_joint(Joint(**item))
    for item in data.get("frames", []):
        robot.add_frame(Frame(**item))
    for item in data.get("cables", []):
        item = dict(item)
        route = CableRoute([Attachment(**point) for point in item.pop("route")])
        if item.get("tension_max") == "inf":
            item["tension_max"] = np.inf
        robot.add_cable(Cable(route=route, **item))
    robot.validate()
    return robot


def save_robot(robot: CableRobot, path: str | Path) -> None:
    Path(path).write_text(json.dumps(robot_to_dict(robot), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_robot(path: str | Path) -> CableRobot:
    return robot_from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
