"""Reusable reference robot constructors used by examples and invariant tests."""

from __future__ import annotations

from itertools import product

import numpy as np

from .model import Attachment, Body, Cable, CableRobot, CableRoute, Frame, Joint, JointType, RobotState, transform


def spatial_cdpr() -> tuple[CableRobot, RobotState]:
    """Eight cables from box corners to a floating rectangular platform."""
    robot = CableRobot("Spatial eight-cable CDPR")
    robot.add_body(Body("outer_frame", fixed=True, visual_size=0.15))
    robot.add_body(Body("moving_platform", mass=2.0, center_of_mass=np.array([0.03, -0.02, 0.04]), visual_size=0.35))
    robot.add_joint(Joint("platform_pose", "outer_frame", "moving_platform", JointType.FLOATING))
    # Unequal platform half-extents avoid the radial geometry that loses torque authority.
    for index, (sx, sy, sz) in enumerate(product((-1, 1), repeat=3)):
        anchor = np.array([1.5 * sx, 1.5 * sy, 1.5 * sz])
        attachment = np.array([0.22 * sx, 0.18 * sy, 0.12 * sz])
        frame_name = f"anchor_{index + 1}"
        robot.add_frame(Frame(frame_name, "outer_frame", transform(translation=anchor)))
        robot.add_cable(Cable(f"cable_{index + 1}", CableRoute([
            Attachment(frame_name, np.zeros(3)), Attachment("moving_platform", attachment),
        ]), tension_min=1.0, tension_max=100.0))
    robot.validate()
    return robot, RobotState(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))


def serial_mechanism() -> tuple[CableRobot, RobotState]:
    """Two articulated links and cables including a base-to-distal attachment."""
    robot = CableRobot("Serial cable-driven two-link mechanism")
    robot.add_body(Body("ground", fixed=True))
    robot.add_body(Body("link_1", mass=1.0, center_of_mass=np.array([0.45, 0.0, 0.0])))
    robot.add_body(Body("link_2", mass=0.6, center_of_mass=np.array([0.35, 0.0, 0.0])))
    robot.add_joint(Joint("shoulder", "ground", "link_1", JointType.REVOLUTE, axis=np.array([0.0, 1.0, 0.0])))
    robot.add_joint(Joint("elbow", "link_1", "link_2", JointType.REVOLUTE, axis=np.array([0.0, 1.0, 0.0]), T_parent_joint=transform(translation=[0.9, 0.0, 0.0])))
    robot.add_frame(Frame("tip", "link_2", transform(translation=[0.7, 0.0, 0.0])))
    routes = [
        ("distal_upper", [Attachment("ground", np.array([-0.3, 0.3, 1.2])), Attachment("tip", np.zeros(3))]),
        ("distal_lower", [Attachment("ground", np.array([-0.3, -0.3, -1.2])), Attachment("tip", np.zeros(3))]),
        ("interlink", [Attachment("link_1", np.array([0.2, 0.2, 0.25])), Attachment("link_2", np.array([0.6, -0.2, 0.15]))]),
        ("routed_distal", [Attachment("ground", np.array([-0.5, -0.3, 0.8])), Attachment("link_1", np.array([0.4, -0.3, 0.2])), Attachment("tip", np.array([0.0, -0.2, 0.0]))]),
    ]
    for name, points in routes:
        robot.add_cable(Cable(name, CableRoute(points), tension_max=80.0))
    robot.validate()
    return robot, RobotState(np.array([0.25, -0.5]))


def _add_routed_arm(robot: CableRobot, mount: str, lengths: tuple[float, ...],
                    *, mount_offset=(0.0, 0.0, 0.0)) -> None:
    """Add a three-link arm with outer-frame and entirely on-robot routes."""
    for index, length in enumerate(lengths, start=1):
        body_name = f"link_{index}"
        robot.add_body(Body(body_name, mass=0.5 / index,
                            center_of_mass=np.array([length / 2, 0.0, 0.0])))
        parent = mount if index == 1 else f"link_{index - 1}"
        offset = mount_offset if index == 1 else (lengths[index - 2], 0.0, 0.0)
        robot.add_joint(Joint(f"arm_joint_{index}", parent, body_name, JointType.REVOLUTE,
                              axis=np.array([0., 1., 0.]),
                              T_parent_joint=transform(translation=offset)))
        robot.add_frame(Frame(f"link_{index}_end", body_name,
                              transform(translation=[length, 0., 0.])))
        for side, sign in (("upper", 1.), ("lower", -1.)):
            robot.add_frame(Frame(f"link_{index}_{side}_guide", body_name,
                                  transform(translation=[0.55 * length, sign * 0.14, sign * 0.16])))
    robot.add_frame(Frame("onboard_start", "link_1",
                          transform(translation=[0.05, -0.20, 0.10])))
    robot.add_frame(Frame("onboard_finish", "link_3",
                          transform(translation=[lengths[-1], -0.20, -0.04])))
    robot.add_frame(Frame("outer_feed", "outer_frame",
                          transform(translation=[-0.45, 0.40, 1.05])))
    robot.add_frame(Frame("outer_return", "outer_frame",
                          transform(translation=[-0.45, -0.40, -1.05])))
    routes = [
        ("outer_to_tip", ("outer_feed", "link_1_upper_guide", "link_2_upper_guide", "link_3_end")),
        ("outer_to_link2", ("outer_return", "link_1_lower_guide", "link_2_end")),
        ("on_robot_routed", ("onboard_start", "link_1_lower_guide", "link_2_lower_guide", "onboard_finish")),
        ("on_robot_direct", ("link_1_upper_guide", "onboard_finish")),
    ]
    for name, frames in routes:
        robot.add_cable(Cable(name, CableRoute([Attachment(frame, np.zeros(3)) for frame in frames]),
                              tension_max=80.0))


def serial_routed_mechanism() -> tuple[CableRobot, RobotState]:
    """Three revolute links: external routing and routes wholly on moving links."""
    robot = CableRobot("Serial robot with external and onboard routing")
    robot.add_body(Body("outer_frame", fixed=True))
    robot.add_body(Body("robot_mount", fixed=True))
    robot.add_joint(Joint("mount_fixture", "outer_frame", "robot_mount", JointType.FIXED))
    _add_routed_arm(robot, "robot_mount", (0.65, 0.50, 0.38))
    robot.analysis_frames = ("robot_mount", "link_1_end", "link_2_end", "link_3_end")
    robot.validate()
    return robot, RobotState(np.array([-0.30, 0.50, -0.40]))


def hybrid_mechanism() -> tuple[CableRobot, RobotState]:
    """A cable-supported floating platform carrying a three-joint routed arm."""
    robot, _ = spatial_cdpr()
    robot.name = "Hybrid floating platform and serial cable-driven arm"
    _add_routed_arm(robot, "moving_platform", (0.50, 0.40, 0.30),
                    mount_offset=(0.22, 0.0, 0.12))
    robot.analysis_frames = ("moving_platform", "link_1_end", "link_2_end", "link_3_end")
    robot.validate()
    return robot, RobotState(np.array([-0.25, 0.0, 0.15, 0.0, 0.0, 0.0,
                                       -0.30, 0.50, -0.40]))


def routing_animation_panels(*, duration: float = 5.0, fps: float = 20.0):
    """Three synchronized demonstration panels, with explicit display geometry."""
    from .simulation import sinusoidal_trajectory
    from .visualization.animation import AnimationPanel, wireframe_box

    spatial, spatial_state = spatial_cdpr()
    serial, serial_state = serial_routed_mechanism()
    hybrid, hybrid_state = hybrid_mechanism()
    definitions = [
        ("spatial", spatial, spatial_state, "Spatial · 6 DOF · 8 cables",
         [0.20, 0.16, 0.10, 0.12, 0.14, 0.20], [0., 1.57, 0.7, 0.2, 1.0, 1.57]),
        ("serial", serial, serial_state, "Serial · 3 joints · 4 routed cables",
         [0.40, 0.60, 0.50], [0., 0.8, 1.7]),
        ("hybrid", hybrid, hybrid_state, "Hybrid · 9 DOF · 12 cables",
         [0.12, 0.08, 0.08, 0.08, 0.12, 0.16, 0.30, 0.45, 0.35],
         [0., 1.57, 0.7, 0.2, 1.0, 1.57, 0., 0.8, 1.7]),
    ]
    panels = {}
    for key, robot, initial, title, amplitude, phase in definitions:
        geometry = {
            "outer_frame": wireframe_box([2.8, 1.2, 2.4], center=[0.75, 0., 0.])
            if key == "serial" else wireframe_box([3., 3., 3.]),
        }
        if "moving_platform" in robot.bodies:
            geometry["moving_platform"] = wireframe_box([0.44, 0.36, 0.24])
        for index in range(1, 4):
            name = f"link_{index}"
            if name in robot.bodies:
                endpoint = robot.frames[f"{name}_end"].T_body_frame[:3, 3]
                geometry[name] = np.array([[np.zeros(3), endpoint]])
        trajectory = sinusoidal_trajectory(robot, initial, amplitude,
                                           duration=duration, fps=fps, phases=phase)
        selected = ("moving_platform",) if key == "spatial" else tuple(f"link_{i}_end" for i in range(1, 4))
        panels[key] = AnimationPanel(robot, trajectory, title, geometry, selected)
    return panels
