"""Regression coverage for the foundation correctness and procedure review."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cablerobot import (
    AllocationStatus, Body, CableRobot, RobotState, allocate_tensions,
    cable_generalized_force, equilibrium_residual, gravity_generalized_force,
    solve_frame_position,
)
from cablerobot.analysis import analyze_wrench_workspace, cable_jacobian_rank
from cablerobot.backends import BackendUnavailableError, MatlabBackend, get_backend
from cablerobot.dynamics.equations import numerical_terms
from cablerobot.examples import serial_routed_mechanism
from cablerobot.io import robot_from_dict, robot_to_dict
from cablerobot.model.transforms import validate_transform
from cablerobot.simulation import KinematicTrajectory, sinusoidal_trajectory
from cablerobot.visualization.animation import AnimationPanel, RobotAnimation


@pytest.mark.parametrize("scale", [1., 1e-7])
def test_feasible_ill_conditioned_problem_never_claims_infeasibility(scale):
    matrix = np.zeros((2, 9))
    matrix[0, :3] = [1., 1., -1.]
    matrix[1, 1] = 1e-5
    matrix *= scale
    witness = np.zeros(9)
    witness[1:3] = 1e5 / scale
    upper = np.full(9, 1e6 / scale)
    np.testing.assert_allclose(matrix @ witness, [0., 1.], atol=1e-12, rtol=0)
    result = allocate_tensions(matrix, [0., 1.], np.zeros(9), upper)
    assert result.status in (AllocationStatus.FEASIBLE, AllocationStatus.NUMERICALLY_UNRESOLVED)


@pytest.mark.parametrize("count", [1, 9])
def test_infeasible_separation_with_unbounded_upper_and_negative_force(count):
    result = allocate_tensions(-np.ones((1, count)), [1.], np.zeros(count), np.full(count, np.inf))
    assert result.status is AllocationStatus.INFEASIBLE


@pytest.mark.parametrize("count", [1, 4, 9])
def test_known_feasible_witnesses_under_matrix_scaling(count):
    rng = np.random.default_rng(724)
    for scale in (1e-9, 1., 1e9):
        matrix = rng.normal(size=(3, count)) * scale
        lower, upper = np.full(count, .5), np.full(count, 5.)
        witness = rng.uniform(lower, upper)
        target = matrix @ witness
        result = allocate_tensions(matrix, target, lower, upper)
        assert result.status in (AllocationStatus.FEASIBLE, AllocationStatus.NUMERICALLY_UNRESOLVED)
        assert np.all(result.tensions >= lower) and np.all(result.tensions <= upper)
        if result.feasible:
            assert result.residual_norm <= 1e-8 * (1 + np.linalg.norm(target))


def test_overflowing_norm_cannot_report_feasibility():
    result = allocate_tensions([[1.]], [1e308], [0.], [1.])
    assert result.status is AllocationStatus.NUMERICAL_FAILURE


@pytest.mark.parametrize("perturbation", ["scale", "bottom_row"])
def test_transform_absolute_tolerances_are_not_relaxed(perturbation):
    matrix = np.eye(4)
    matrix[0, 0] += 1e-6 if perturbation == "scale" else 0.
    matrix[3, 3] += 1e-6 if perturbation == "bottom_row" else 0.
    with pytest.raises(ValueError):
        validate_transform(matrix)


def test_inertia_symmetry_uses_absolute_tolerance():
    inertia = np.array([[2., .5, 0.], [.5 + 1e-6, 2., 0.], [0., 0., 1.]])
    with pytest.raises(ValueError, match="symmetric"):
        Body("invalid", inertia=inertia)


def test_provider_mass_symmetry_uses_absolute_tolerance():
    robot, state = serial_routed_mechanism()
    matrix = np.eye(3)
    matrix[0, 1], matrix[1, 0] = .5, .5 + 1e-6
    provider = SimpleNamespace(mass_matrix=lambda *_: matrix, bias_force=lambda *_: np.zeros(3))
    with pytest.raises(ValueError, match="symmetric"):
        numerical_terms(robot, state, provider)


@pytest.mark.parametrize("field,value", [("q", np.array([np.nan])), ("qd", np.array([np.inf])), ("qd", np.zeros(2))])
def test_mutated_state_is_revalidated(slider, field, value):
    robot, state = slider
    setattr(state, field, value)
    with pytest.raises(ValueError):
        robot.cable_lengths(state)


@pytest.mark.parametrize("field,value", [("T_body_frame", np.diag([1., 1., 1., 1. + 1e-6])), ("body", "earth")])
def test_canonical_body_frame_cannot_be_repurposed(slider, field, value):
    robot, state = slider
    setattr(robot.frames["carriage"], field, value)
    with pytest.raises(ValueError, match="body frame"):
        robot.cable_lengths(state)
    with pytest.raises(ValueError, match="body frame"):
        robot_to_dict(robot)


@pytest.mark.parametrize("mutation", ["unknown_parent", "duplicate_joint", "wrong_frame_key", "unknown_route_frame"])
def test_mutated_topology_rejected(slider, mutation):
    robot, _ = slider
    if mutation == "unknown_parent":
        robot.joints[0].parent = "missing"
    elif mutation == "duplicate_joint":
        robot.joints.append(robot.joints[0])
    elif mutation == "wrong_frame_key":
        robot.frames["renamed"] = robot.frames.pop("carriage")
    else:
        robot.cables[0].route.points[0].frame = "missing"
    with pytest.raises(ValueError):
        robot.validate()


def test_reordered_joint_coordinate_blocks_rejected():
    robot, state = serial_routed_mechanism()
    robot.joints.reverse()
    with pytest.raises(ValueError, match="coordinate"):
        robot.cable_lengths(state)


def test_gravity_validates_even_without_massive_bodies():
    with pytest.raises(ValueError, match="no bodies"):
        gravity_generalized_force(CableRobot("empty"), RobotState([]))


def test_json_fixed_flag_must_be_boolean(slider):
    robot, _ = slider
    data = robot_to_dict(robot)
    data["bodies"][0]["fixed"] = "false"
    with pytest.raises(ValueError, match="fixed"):
        robot_from_dict(data)


@pytest.mark.parametrize("tension", [-1., np.nan, np.inf])
def test_physical_tensions_must_be_finite_and_nonnegative(slider, tension):
    robot, state = slider
    with pytest.raises(ValueError, match="tensions"):
        cable_generalized_force(robot, state, [tension])


def test_external_load_must_be_finite(slider):
    robot, state = slider
    with pytest.raises(ValueError, match="external_load"):
        equilibrium_residual(robot, state, [1.], [np.nan])


@pytest.mark.parametrize("options", [
    {"tolerance": np.nan}, {"tolerance": np.inf}, {"damping": np.nan},
    {"damping": np.inf}, {"max_iterations": 1.5}, {"max_iterations": True},
])
def test_invalid_solver_options_rejected(slider, options):
    robot, state = slider
    with pytest.raises(ValueError):
        solve_frame_position(robot, "carriage", [0., 0., .5], state, **options)


def test_nonfinite_ik_position_is_invalid_input(slider):
    robot, state = slider
    with pytest.raises(ValueError, match="desired_position"):
        solve_frame_position(robot, "carriage", [0., np.nan, .5], state)


@pytest.mark.parametrize("available", [False, True])
def test_matlab_selection_cannot_claim_implemented_backend(slider, monkeypatch, available):
    monkeypatch.setattr(MatlabBackend, "available", staticmethod(lambda: available))
    robot, state = slider
    with pytest.raises(BackendUnavailableError):
        get_backend("matlab")
    with pytest.raises(BackendUnavailableError):
        analyze_wrench_workspace(robot, state, backend="matlab")
    assert get_backend("numpy").name == "numpy"


def test_empty_cable_jacobian_has_zero_rank(pendulum):
    robot, state = pendulum
    assert cable_jacobian_rank(robot, state) == 0


def test_small_timeline_error_is_not_hidden_by_relative_tolerance():
    with pytest.raises(ValueError, match="uniformly"):
        KinematicTrajectory(np.array([0., .1000005]), np.zeros((2, 1)), np.zeros((2, 1)), 10.)


def test_frame_count_must_match_requested_duration():
    robot, state = serial_routed_mechanism()
    with pytest.raises(ValueError, match="integer"):
        sinusoidal_trajectory(robot, state, [.1, .1, .1], duration=5.00004, fps=20.)


def _export_with_saver(saver, fps=20.):
    robot, state = serial_routed_mechanism()
    trajectory = sinusoidal_trajectory(robot, state, [.1, .1, .1], duration=1., fps=fps)
    return RobotAnimation(None, SimpleNamespace(save=saver), (AnimationPanel(robot, trajectory),), (), None)


def test_failed_animation_save_preserves_destination_and_removes_temp(tmp_path):
    output = tmp_path / "existing.gif"
    output.write_bytes(b"existing animation")

    def failing_save(filename, **kwargs):
        Path(filename).write_bytes(b"incomplete encoding")
        raise RuntimeError("encoder failed")

    rendered = _export_with_saver(failing_save)
    with pytest.raises(RuntimeError, match="encoder failed"):
        rendered.save(output)
    assert output.read_bytes() == b"existing animation"
    assert sorted(p.name for p in tmp_path.iterdir()) == ["existing.gif"]


def test_successful_animation_save_atomically_replaces_destination(tmp_path):
    output = tmp_path / "existing.gif"
    output.write_bytes(b"old")

    def successful_save(filename, **kwargs):
        assert Path(filename) != output
        Path(filename).write_bytes(b"complete animation")

    _export_with_saver(successful_save).save(output)
    assert output.read_bytes() == b"complete animation"
    assert sorted(p.name for p in tmp_path.iterdir()) == ["existing.gif"]


@pytest.mark.parametrize("fps", [30., 120.])
def test_gif_rejects_unrepresentable_frame_duration(tmp_path, fps):
    called = []
    rendered = _export_with_saver(lambda *args, **kwargs: called.append(args), fps=fps)
    with pytest.raises(ValueError, match="10 ms"):
        rendered.save(tmp_path / "invalid.gif")
    assert called == []
    assert not list(tmp_path.iterdir())


def test_representable_gif_rate_does_not_truncate_milliseconds(tmp_path):
    recorded = []

    def save(filename, *, writer, **kwargs):
        recorded.append(int(1000 / writer.fps))
        Path(filename).write_bytes(b"complete")

    robot, state = serial_routed_mechanism()
    trajectory = sinusoidal_trajectory(robot, state, [.1, .1, .1], duration=.3, fps=100 / 3)
    rendered = RobotAnimation(None, SimpleNamespace(save=save), (AnimationPanel(robot, trajectory),), (), None)
    rendered.save(tmp_path / "exact.gif")
    assert recorded == [30]
