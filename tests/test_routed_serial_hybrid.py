import numpy as np
import pytest

from cablerobot import Attachment, Cable, CableRoute, RobotState
from cablerobot.analysis import cable_wrench_matrix, frame_twist_jacobian
from cablerobot.examples import hybrid_mechanism, serial_routed_mechanism
from cablerobot.io import robot_from_dict, robot_to_dict
from cablerobot.kinematics import cable_route_points


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_both_route_types_have_body_attached_moving_guides(factory):
    robot, state = factory()
    by_name = {c.name: c for c in robot.cables}
    external = by_name["outer_to_tip"]
    onboard = by_name["on_robot_routed"]
    assert len(external.route.points) == 4 and len(onboard.route.points) == 4
    assert robot.frames[external.route.points[0].frame].body == "outer_frame"
    assert all(not robot.bodies[robot.frames[p.frame].body].fixed for p in onboard.route.points)
    changed = RobotState(state.q.copy())
    changed.q[-2:] += [0.3, -0.2]
    for cable in (external, onboard):
        before = cable_route_points(robot, cable, state)
        after = cable_route_points(robot, cable, changed)
        np.testing.assert_allclose(before[0], after[0], atol=1e-12)
        assert np.linalg.norm(before[-1] - after[-1]) > 0.03
        lengths = robot.cable_lengths(state)
        reference = np.linalg.norm(np.diff(before, axis=0), axis=1).sum()
        assert lengths[robot.cables.index(cable)] == pytest.approx(reference, abs=1e-12)


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_onboard_routes_invariant_under_shared_shoulder_motion(factory):
    robot, state = factory()
    new_q = state.q.copy()
    new_q[-3] += 0.8
    before = robot.cable_lengths(state)
    after = robot.cable_lengths(RobotState(new_q))
    onboard = [i for i, cable in enumerate(robot.cables) if cable.name.startswith("on_robot")]
    external = [i for i, cable in enumerate(robot.cables) if cable.name.startswith("outer_to")]
    np.testing.assert_allclose(after[onboard], before[onboard], atol=1e-12)
    assert np.linalg.norm(after[external] - before[external]) > 0.03
    np.testing.assert_allclose(robot.cable_jacobian(state)[onboard, -3], 0., atol=1e-8)


def test_hybrid_onboard_routes_invariant_under_floating_platform_motion():
    robot, state = hybrid_mechanism()
    changed = state.q.copy()
    changed[:6] += [0.3, -0.2, 0.1, 0.2, -0.3, 0.4]
    onboard = [i for i, cable in enumerate(robot.cables) if cable.name.startswith("on_robot")]
    external = [i for i, cable in enumerate(robot.cables) if cable.name.startswith("outer_to")]
    np.testing.assert_allclose(robot.cable_lengths(RobotState(changed))[onboard],
                               robot.cable_lengths(state)[onboard], atol=1e-12)
    np.testing.assert_allclose(robot.cable_jacobian(state)[onboard, :6], 0., atol=1e-8)
    assert np.linalg.norm(robot.cable_lengths(RobotState(changed))[external] -
                          robot.cable_lengths(state)[external]) > 0.05


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_same_body_route_rigid_motion_invariance(factory):
    robot, state = factory()
    cable = Cable("local_only", CableRoute([
        Attachment("link_3", np.array([0., 0., 0.])),
        Attachment("link_3", np.array([0.2, 0.1, 0.])),
        Attachment("link_3", np.array([0.3, 0.1, 0.2])),
    ]))
    robot.add_cable(cable)
    expected = 2 * np.sqrt(0.05)
    for offset in (0., 0.3, -0.7):
        assert robot.cable_lengths(RobotState(state.q + offset))[-1] == pytest.approx(expected, abs=1e-12)
    np.testing.assert_allclose(robot.cable_jacobian(state)[-1], 0., atol=1e-8)


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_routed_jacobian_virtual_work_and_intermediate_body_wrenches(factory):
    robot, state = factory()
    J = robot.cable_jacobian(state)
    velocity = np.linspace(-0.2, 0.3, robot.dof)
    h = 2e-5
    rate = (robot.cable_lengths(RobotState(state.q + h * velocity)) -
            robot.cable_lengths(RobotState(state.q - h * velocity))) / (2 * h)
    np.testing.assert_allclose(J @ velocity, rate, atol=5e-8, rtol=1e-6)
    tensions = np.linspace(1., 8., robot.cable_count)
    force = robot.cable_force_matrix(state) @ tensions
    assert force @ velocity == pytest.approx(-tensions @ rate, abs=2e-7)
    recovered = np.zeros_like(robot.cable_force_matrix(state))
    for name in robot.bodies:
        recovered += frame_twist_jacobian(robot, state, name).T @ cable_wrench_matrix(robot, state, name)
    np.testing.assert_allclose(recovered, -J.T, atol=5e-8, rtol=1e-6)


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_new_topologies_round_trip_without_special_geometry(factory):
    robot, state = factory()
    restored = robot_from_dict(robot_to_dict(robot))
    assert restored.dof == robot.dof
    assert restored.cable_count == robot.cable_count
    np.testing.assert_allclose(restored.cable_lengths(state), robot.cable_lengths(state), atol=1e-12)
    np.testing.assert_allclose(restored.cable_jacobian(state), robot.cable_jacobian(state), atol=1e-8)
