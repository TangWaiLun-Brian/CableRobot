import json

import numpy as np
import pytest

from cablerobot import RobotState, solve_configuration_from_lengths, solve_frame_pose, solve_frame_position
from cablerobot.analysis import analyze_wrench_workspace, available_generalized_force_set, cable_jacobian_rank, cable_manipulability
from cablerobot.backends import BackendUnavailableError, MatlabBackend, NumPyBackend, TensionProblem, get_backend
from cablerobot.dynamics import FullDynamicsDeferredError, forward_dynamics, inverse_dynamics, static_generalized_force_required
from cablerobot.examples import serial_mechanism, spatial_cdpr
from cablerobot.io import load_robot, robot_from_dict, robot_to_dict, save_robot
from cablerobot.kinematics import SolverStatus


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_lengths_to_configuration_uses_common_geometry(factory):
    robot, target = factory()
    initial = RobotState(target.q + 0.04)
    result = solve_configuration_from_lengths(robot, robot.cable_lengths(target), initial)
    assert result.converged
    np.testing.assert_allclose(result.state.q, target.q, atol=1e-6)


def test_selected_frame_pose_solver():
    robot, initial = spatial_cdpr()
    target = RobotState([0.1, -0.15, 0.2, 0.2, -0.1, 0.15])
    T_target = robot.frame_transform("moving_platform", target)
    result = solve_frame_pose(robot, "moving_platform", T_target, initial)
    assert result.converged
    np.testing.assert_allclose(robot.frame_transform("moving_platform", result.state), T_target, atol=1e-8)


def test_serial_frame_position_solver():
    robot, target = serial_mechanism()
    position = robot.frame_transform("tip", target)[:3, 3]
    result = solve_frame_position(robot, "tip", position, RobotState(target.q + 0.08))
    assert result.converged
    np.testing.assert_allclose(robot.frame_transform("tip", result.state)[:3, 3], position, atol=1e-8)


def test_unreachable_target_reports_nonconvergence(slider):
    robot, state = slider
    result = solve_frame_position(robot, "carriage", [1., 0., 0.5], state, max_iterations=3)
    assert result.status is SolverStatus.MAX_ITERATIONS
    assert not result.converged


def test_numpy_backend_problem_result_contract():
    problem = TensionProblem(np.array([[1.]]), np.array([3.]), np.array([0.]), np.array([5.]))
    result = NumPyBackend().solve_tension_problem(problem)
    assert result.feasible
    np.testing.assert_allclose(result.tensions, [3.], atol=1e-10)
    assert get_backend("numpy").name == "numpy"
    with pytest.raises(ValueError, match="unknown backend"):
        get_backend("bogus")


def test_matlab_unavailable_is_clear(monkeypatch):
    import cablerobot.backends.matlab_backend as adapter

    def missing_module(name):
        raise ModuleNotFoundError("no matlab")

    monkeypatch.setattr(adapter.importlib.util, "find_spec", missing_module)
    assert not MatlabBackend.available()
    problem = TensionProblem(np.ones((1, 1)), np.ones(1), np.zeros(1), np.ones(1))
    with pytest.raises(BackendUnavailableError, match="not installed"):
        MatlabBackend().solve_tension_problem(problem)


def test_matlab_installed_stub_is_explicit(monkeypatch):
    monkeypatch.setattr(MatlabBackend, "available", staticmethod(lambda: True))
    problem = TensionProblem(np.ones((1, 1)), np.ones(1), np.zeros(1), np.ones(1))
    with pytest.raises(BackendUnavailableError, match="adapter boundary"):
        MatlabBackend().solve_tension_problem(problem)


def test_force_set_and_pointwise_backend_analysis(slider):
    robot, state = slider
    force_set = available_generalized_force_set(robot, state)
    np.testing.assert_allclose(force_set.vertices, [[0.], [50.]], atol=1e-7)
    analysis = analyze_wrench_workspace(robot, state, target_generalized_force=np.array([20.]))
    assert analysis.backend == "numpy" and analysis.allocation.feasible
    with pytest.raises(BackendUnavailableError):
        analyze_wrench_workspace(robot, state, backend="matlab", target_generalized_force=np.array([20.]))


def test_rank_manipulability():
    robot, state = spatial_cdpr()
    assert cable_jacobian_rank(robot, state) == 6
    assert cable_manipulability(robot, state) > 0.


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_json_round_trip_preserves_numerics_and_topology(factory, tmp_path):
    robot, state = factory()
    robot.analysis_frames = tuple(robot.bodies)
    path = tmp_path / "robot.json"
    save_robot(robot, path)
    restored = load_robot(path)
    assert restored.analysis_frames == robot.analysis_frames
    assert restored.dof == robot.dof and restored.cable_count == robot.cable_count
    assert [(j.parent, j.child) for j in restored.joints] == [(j.parent, j.child) for j in robot.joints]
    np.testing.assert_allclose(restored.cable_lengths(state), robot.cable_lengths(state), atol=1e-12)
    np.testing.assert_allclose(restored.cable_jacobian(state), robot.cable_jacobian(state), atol=1e-9)


def test_json_unbounded_tension_and_schema_validation(slider):
    robot, _ = slider
    robot.cables[0].tension_max = np.inf
    text = json.dumps(robot_to_dict(robot), allow_nan=False)
    restored = robot_from_dict(json.loads(text))
    assert np.isposinf(restored.cables[0].tension_max)
    with pytest.raises(ValueError, match="schema_version"):
        robot_from_dict({"schema_version": 999})


def test_static_dynamics_special_case_and_deferred_interfaces(slider):
    robot, state = slider
    np.testing.assert_allclose(static_generalized_force_required(robot, state), [19.62], atol=1e-8)
    with pytest.raises(FullDynamicsDeferredError):
        forward_dynamics(robot, state, [20.])
    with pytest.raises(FullDynamicsDeferredError):
        inverse_dynamics(robot, state, [0.])
