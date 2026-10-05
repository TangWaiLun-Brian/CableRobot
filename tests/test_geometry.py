import numpy as np
import pytest

from cablerobot import Attachment, Body, Cable, CableRobot, CableRoute, Frame, Joint, RobotState, transform
from cablerobot.examples import serial_mechanism, spatial_cdpr
from cablerobot.kinematics import frame_position, numerical_jacobian
from cablerobot.model.transforms import inverse_transform, rotation_about_axis, rotation_from_rotvec, rotvec_from_rotation, transform_point, validate_transform


def test_rigid_transform_composition_and_inverse():
    a = transform(rotation_about_axis([0, 0, 1], np.pi / 2), [1, 2, 3])
    b = transform(translation=[2, 0, 1])
    p = np.array([0.2, 0.3, 0.4])
    np.testing.assert_allclose(transform_point(a @ b, p), transform_point(a, transform_point(b, p)), atol=1e-12)
    np.testing.assert_allclose(inverse_transform(a) @ a, np.eye(4), atol=1e-12)


@pytest.mark.parametrize("r", [[0., 0., 0.], [0.2, -0.4, 0.1], [np.pi, 0., 0.]])
def test_so3_round_trip(r):
    R = rotation_from_rotvec(r)
    np.testing.assert_allclose(rotation_from_rotvec(rotvec_from_rotation(R)), R, atol=1e-8)


def test_frame_position_consistency(slider):
    robot, state = slider
    robot.add_frame(Frame("sensor", "carriage", transform(translation=[0.2, -0.1, 0.3])))
    np.testing.assert_allclose(frame_position(robot, "sensor", state), [0.2, -0.1, 0.8], atol=1e-12)


def test_known_slider_lengths_and_jacobian(slider):
    robot, state = slider
    np.testing.assert_allclose(robot.cable_lengths(state), [1.5], atol=1e-12)
    np.testing.assert_allclose(robot.cable_jacobian(state), [[-1]], atol=1e-9)
    np.testing.assert_allclose(robot.cable_force_matrix(state), [[1]], atol=1e-9)


def test_routed_length_and_analytic_derivative(slider):
    robot, state = slider
    robot.cables.clear()
    robot.add_cable(Cable("routed", CableRoute([
        Attachment("earth", np.array([0., 0., 0.])),
        Attachment("earth", np.array([1., 0., 0.])),
        Attachment("carriage", np.zeros(3)),
    ])))
    expected = 1 + np.sqrt(1 + state.q[0] ** 2)
    np.testing.assert_allclose(robot.cable_lengths(state), [expected], atol=1e-12)
    np.testing.assert_allclose(robot.cable_jacobian(state), [[state.q[0] / np.sqrt(1 + state.q[0] ** 2)]], atol=1e-8)


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_jacobian_against_independent_five_point_difference(factory):
    robot, state = factory()
    state = RobotState(state.q + np.linspace(0.05, 0.15, robot.dof))
    h = 3e-5
    reference = np.empty((robot.cable_count, robot.dof))
    for column in range(robot.dof):
        delta = np.zeros(robot.dof)
        delta[column] = h
        length = lambda shift: robot.cable_lengths(RobotState(state.q + shift * delta))
        reference[:, column] = (-length(2) + 8 * length(1) - 8 * length(-1) + length(-2)) / (12 * h)
    np.testing.assert_allclose(robot.cable_jacobian(state), reference, atol=2e-8, rtol=2e-7)


def test_serial_distal_position_matches_planar_geometry():
    robot, state = serial_mechanism()
    shoulder, elbow = state.q
    expected = [0.9 * np.cos(shoulder) + 0.7 * np.cos(shoulder + elbow), 0,
                -0.9 * np.sin(shoulder) - 0.7 * np.sin(shoulder + elbow)]
    np.testing.assert_allclose(frame_position(robot, "tip", state), expected, atol=1e-12)
    assert robot.cables[0].route.points[0].frame == "ground"
    assert robot.cables[0].route.points[-1].frame == "tip"


def test_cdpr_construction_and_full_rank():
    robot, state = spatial_cdpr()
    assert robot.dof == 6 and robot.cable_count == 8
    assert np.linalg.matrix_rank(robot.cable_jacobian(state), tol=1e-8) == 6


def test_zero_dof_fixed_cable():
    robot = CableRobot("fixed segment")
    robot.add_body(Body("wall", fixed=True))
    robot.add_cable(Cable("tie", CableRoute([Attachment("wall", np.zeros(3)), Attachment("wall", np.ones(3))])))
    assert robot.cable_jacobian(robot.zero_state()).shape == (1, 0)
    np.testing.assert_allclose(robot.cable_lengths(robot.zero_state()), [np.sqrt(3)], atol=1e-12)


def test_zero_length_segment_rejects_jacobian(slider):
    robot, _ = slider
    with pytest.raises(ValueError, match="zero-length"):
        robot.cable_jacobian(RobotState([2.]))


@pytest.mark.parametrize("invalid", [np.diag([1, 1, -1, 1]), np.full((4, 4), np.nan), np.ones((3, 3))])
def test_invalid_rigid_transforms(invalid):
    with pytest.raises(ValueError):
        validate_transform(invalid)


def test_bad_state_shape(slider):
    robot, _ = slider
    with pytest.raises(ValueError, match="requires 1"):
        robot.body_transforms(RobotState([1, 2]))


def test_topology_cycle():
    robot = CableRobot("cycle")
    robot.add_body(Body("a"))
    robot.add_body(Body("b"))
    robot.add_joint(Joint("one", "a", "b", "fixed"))
    robot.add_joint(Joint("two", "b", "a", "fixed"))
    with pytest.raises(ValueError, match="root"):
        robot.validate()


def test_unknown_attachment_frame(slider):
    robot, _ = slider
    with pytest.raises(ValueError, match="unknown frames"):
        robot.add_cable(Cable("bad", CableRoute([Attachment("earth", np.zeros(3)), Attachment("missing", np.zeros(3))])))


def test_multiple_parents_rejected(slider):
    robot, _ = slider
    with pytest.raises(ValueError, match="parent joint"):
        robot.add_joint(Joint("another", "earth", "carriage", "fixed"))


def test_nonfinite_state_and_invalid_fd_step():
    with pytest.raises(ValueError, match="finite"):
        RobotState([np.nan])
    with pytest.raises(ValueError, match="step"):
        numerical_jacobian(lambda q: q, np.array([1.]), 0.)


def test_fixed_joint_and_joint_frame_axis_convention():
    robot = CableRobot("offset joints")
    robot.add_body(Body("root", fixed=True))
    robot.add_body(Body("fixture", fixed=True))
    robot.add_body(Body("moving"))
    robot.add_joint(Joint("offset", "root", "fixture", "fixed", T_parent_joint=transform(translation=[2, 0, 0])))
    robot.add_joint(Joint("slide", "fixture", "moving", "prismatic", axis=[1, 0, 0],
                          T_parent_joint=transform(rotation_about_axis([0, 0, 1], np.pi / 2)),
                          T_joint_child=transform(translation=[0, 0, 0.3])))
    np.testing.assert_allclose(robot.frame_transform("moving", RobotState([0.5]))[:3, 3], [2, 0.5, 0.3], atol=1e-12)


def test_fixed_body_with_moving_ancestor_rejected(slider):
    robot, state = slider
    robot.bodies["carriage"].fixed = True
    with pytest.raises(ValueError, match="moving ancestor"):
        robot.body_transforms(state)
