"""Independent physics/objective checks for Milestone 2; foundation tests unchanged."""

from itertools import product

import numpy as np
import pytest

from cablerobot import (
    AllocationStatus, RobotState, allocate_reference_tensions, allocate_tensions,
    equilibrium_residual, gravity_generalized_force, solve_equilibrium_tensions,
)
from cablerobot.analysis import cable_wrench_matrix
from cablerobot.examples import spatial_cdpr, serial_routed_mechanism, hybrid_mechanism


def test_analytic_gravity_and_external_load(slider):
    robot, state = slider
    result = solve_equilibrium_tensions(robot, state, [-5.], reference_tension=7.)
    assert result.feasible and result.optimality_verified
    np.testing.assert_allclose(result.tensions, [24.62], atol=1e-7, rtol=0)
    np.testing.assert_allclose(result.residual, equilibrium_residual(robot, state, result.tensions, [-5.]), atol=1e-10)
    np.testing.assert_allclose(result.lower_margin, result.tensions, atol=1e-10)
    np.testing.assert_allclose(result.upper_margin, 50 - result.tensions, atol=1e-10)
    command = result.tension_command
    command[0] = 0
    assert result.tensions[0] > 24  # a consumer cannot mutate the proposed result via the command


def test_explicit_gravity_exclusion(slider):
    robot, state = slider
    result = solve_equilibrium_tensions(robot, state, [-4.], include_gravity=False)
    np.testing.assert_allclose(result.tensions, [4.], atol=1e-8, rtol=0)


@pytest.mark.parametrize("weights,expected,objective", [(None, [4, 6], 4), ([1, 2], [5.2, 4.8], 6.4)])
def test_redundancy_reference_and_squared_weight_convention(weights, expected, objective):
    result = allocate_reference_tensions([[1, 1]], [10], [0, 0], [100, 100],
                                         reference_tension=[2, 4], weights=weights)
    assert result.feasible
    np.testing.assert_allclose(result.tensions, expected, atol=1e-9, rtol=0)
    assert result.objective_value == pytest.approx(objective, abs=1e-9)
    assert result.duality_gap < 1e-9
    # Interior stationarity: W^2(t-ref) = B.T lambda, not W(t-ref).
    np.testing.assert_allclose(result.weights**2 * (result.tensions - result.reference_tension),
                               np.ones(2) * result.dual_multiplier[0], atol=1e-9, rtol=0)


@pytest.mark.parametrize("reference", [3., [3., 3.]])
def test_scalar_vector_reference_and_default_minimum_norm(reference):
    result = allocate_reference_tensions([[1, 1]], [8], [0, 0], [100, 100], reference_tension=reference)
    default = allocate_reference_tensions([[1, 1]], [8], [0, 0], [100, 100])
    np.testing.assert_allclose(result.tensions, [4, 4], atol=1e-9)
    np.testing.assert_allclose(default.tensions, [4, 4], atol=1e-9)
    assert default.objective_value == pytest.approx(16.)


def test_reference_outside_box_and_both_active_bounds():
    result = allocate_reference_tensions([[1, 1]], [10], [2, 0], [6, 8], reference_tension=[0, 20])
    assert result.feasible
    np.testing.assert_allclose(result.tensions, [2, 8], atol=1e-9, rtol=0)
    assert result.active_lower.tolist() == [True, False]
    assert result.active_upper.tolist() == [False, True]
    assert result.objective_value == pytest.approx(74.)
    gradient = result.weights**2 * (result.tensions - result.reference_tension) - result.dual_multiplier[0]
    assert gradient[0] >= -1e-9 and gradient[1] <= 1e-9  # correct bound KKT signs


def test_coordinate_and_cable_permutations_preserve_optimum():
    matrix = np.array([[1., 1., 0.], [0., 1., 1.]])
    target, lower, upper = np.array([1., 2.]), np.zeros(3), np.full(3, 20.)
    reference, weights = np.array([10., 9., 8.]), np.array([1., 2., 3.])
    result = allocate_reference_tensions(matrix, target, lower, upper, reference_tension=reference, weights=weights)
    order = np.array([2, 0, 1])
    # Orthogonal row mapping changes no complete equality or residual norm.
    rotation = np.array([[0., -1.], [1., 0.]])
    changed = allocate_reference_tensions(rotation @ matrix[:, order], rotation @ target,
                                          lower[order], upper[order], reference_tension=reference[order], weights=weights[order])
    scaled = allocate_reference_tensions(matrix, target, lower, upper, reference_tension=reference, weights=10*weights)
    assert result.feasible and changed.feasible and scaled.feasible
    np.testing.assert_allclose(changed.tensions, result.tensions[order], atol=1e-8, rtol=0)
    np.testing.assert_allclose(scaled.tensions, result.tensions, atol=1e-8, rtol=0)
    assert scaled.objective_value == pytest.approx(100*result.objective_value, abs=1e-6)


def test_fixed_and_infinite_bounds():
    result = allocate_reference_tensions([[1, 1]], [7], [2, 0], [2, np.inf], reference_tension=1)
    assert result.feasible
    np.testing.assert_allclose(result.tensions, [2, 5], atol=1e-9, rtol=0)
    assert result.active_lower[0] and result.active_upper[0]
    assert np.isinf(result.upper_margin[1]) and not result.active_upper[1]


@pytest.mark.parametrize("target", [-1., 11.])
def test_certified_infeasibility_returns_no_command(target):
    result = allocate_reference_tensions([[1]], [target], [1], [10], reference_tension=5)
    assert result.status is AllocationStatus.INFEASIBLE
    assert not result.feasible and not result.equilibrium_feasible
    assert result.tension_command is None
    np.testing.assert_allclose(result.residual, np.array([[1]]) @ result.tensions - target, atol=0)
    assert result.residual_norm > 0.9


def test_rank_deficient_equalities_preserve_all_rows():
    result = allocate_reference_tensions([[1, 1], [2, 2], [0, 0]], [6, 12, 0], [0, 0], [10, 10], reference_tension=[1, 3])
    assert result.feasible
    np.testing.assert_allclose(result.tensions, [2, 4], atol=1e-8, rtol=0)
    impossible = allocate_reference_tensions([[1, 1], [2, 2]], [6, 13], [0, 0], [10, 10])
    assert impossible.status is AllocationStatus.INFEASIBLE
    assert impossible.residual.shape == (2,)


@pytest.mark.parametrize("matrix,target,lower,upper,expected,status", [
    (np.zeros((0, 2)), [], [1, 1], [10, 10], [3, 3], AllocationStatus.FEASIBLE),
    (np.zeros((2, 0)), [0, 0], [], [], [], AllocationStatus.FEASIBLE),
    (np.zeros((1, 0)), [1], [], [], [], AllocationStatus.INFEASIBLE),
    (np.zeros((1, 2)), [0], [1, 1], [2, 2], [2, 2], AllocationStatus.FEASIBLE),
])
def test_empty_and_zero_mappings(matrix, target, lower, upper, expected, status):
    result = allocate_reference_tensions(matrix, target, lower, upper, reference_tension=3)
    assert result.status is status
    np.testing.assert_allclose(result.tensions, expected, atol=1e-10, rtol=0)


def test_nine_cable_ill_conditioned_feasible_witness_is_not_called_infeasible():
    matrix = np.zeros((2, 9))
    matrix[0, :3] = [1, 1, -1]
    matrix[1, 1] = 1e-5
    result = allocate_reference_tensions(matrix, [0, 1], np.zeros(9), np.full(9, 1e6))
    assert result.feasible
    np.testing.assert_allclose(result.tensions[:3], [0, 1e5, 1e5], atol=1e-3, rtol=0)
    assert result.residual_norm < 1e-7


def test_iteration_exhaustion_cannot_report_fallback_as_reference_optimum():
    result = allocate_reference_tensions([[1, 1, 0], [0, 1, 1]], [1, 2], [0]*3, [20]*3,
                                         reference_tension=10, max_iterations=1)
    assert result.status is AllocationStatus.NUMERICALLY_UNRESOLVED
    assert result.equilibrium_feasible  # foundation witness, not QP optimality
    assert not result.optimality_verified and result.tension_command is None


def test_svd_failure_and_nonfinite_arithmetic_are_explicit(monkeypatch):
    def failure(*args, **kwargs):
        raise np.linalg.LinAlgError("forced SVD failure")
    monkeypatch.setattr(np.linalg, "svd", failure)
    result = allocate_reference_tensions([[1]], [5], [0], [10], reference_tension=2)
    assert result.status is AllocationStatus.NUMERICAL_FAILURE
    assert "forced SVD failure" in result.message
    assert result.tension_command is None
    overflow = allocate_reference_tensions([[1]], [1], [0], [10], reference_tension=1e200)
    assert overflow.status is AllocationStatus.NUMERICAL_FAILURE
    assert overflow.objective_value is None


@pytest.mark.parametrize("kwargs", [
    {"reference_tension": -1}, {"reference_tension": np.inf}, {"reference_tension": [1, 2]},
    {"reference_tension": [[1]]}, {"weights": 0}, {"weights": -1}, {"weights": np.nan},
    {"weights": [1, 2]}, {"weights": [[1]]}, {"tolerance": 0}, {"tolerance": np.inf},
    {"max_iterations": 0}, {"max_iterations": 1.5}, {"max_iterations": True},
])
def test_invalid_objective_and_numerical_options(kwargs):
    with pytest.raises(ValueError):
        allocate_reference_tensions([[1]], [1], [0], [10], **kwargs)


@pytest.mark.parametrize("matrix,target,lower,upper", [
    ([1], [1], [0], [10]), ([[1]], [1, 2], [0], [10]), ([[1]], [1], [0, 0], [10]),
    ([[np.nan]], [1], [0], [10]), ([[1]], [np.inf], [0], [10]),
    ([[1]], [1], [-1], [10]), ([[1]], [1], [2], [1]),
    ([[1]], [1], [0], [np.nan]), ([[1]], [1], [np.inf], [np.inf]),
])
def test_invalid_numerical_problem(matrix, target, lower, upper):
    with pytest.raises(ValueError):
        allocate_reference_tensions(matrix, target, lower, upper)


@pytest.mark.parametrize("load", [[1, 2], [np.nan], [np.inf]])
def test_invalid_applied_load(slider, load):
    with pytest.raises(ValueError):
        solve_equilibrium_tensions(*slider, load)


def _small_face_oracle(matrix, target, lower, upper, reference, weights):
    """Independent exhaustive equality-face oracle, only for four-cable tests."""
    best = np.inf
    for states in product((0, 1, 2), repeat=len(lower)):
        free = np.array(states) == 0
        values = np.where(np.array(states) == 1, lower, upper).copy()
        rhs = target - matrix[:, ~free] @ values[~free] - matrix[:, free] @ reference[free]
        change = np.linalg.lstsq(matrix[:, free] / weights[free], rhs, rcond=None)[0]
        values[free] = reference[free] + change / weights[free]
        if (np.linalg.norm(matrix @ values - target) > 1e-8
                or np.any(values < lower - 1e-9) or np.any(values > upper + 1e-9)):
            continue
        best = min(best, 0.5 * np.linalg.norm(weights * (values - reference))**2)
    return best


@pytest.mark.parametrize("seed", range(10))
def test_weighted_optimum_against_independent_small_face_oracle(seed):
    rng = np.random.default_rng(seed)
    matrix = rng.normal(size=(2, 4))
    lower, upper = np.ones(4), np.full(4, 5.)
    target = matrix @ rng.uniform(lower, upper)
    reference, weights = rng.uniform(0, 10, 4), rng.uniform(0.5, 3, 4)
    result = allocate_reference_tensions(matrix, target, lower, upper, reference_tension=reference, weights=weights)
    assert result.feasible, result.message
    objective = _small_face_oracle(matrix, target, lower, upper, reference, weights)
    assert result.objective_value == pytest.approx(objective, abs=2e-7, rel=1e-8)
    assert result.residual_norm < 1e-7


def test_spatial_offset_com_full_physical_wrench_at_nonzero_rotation():
    robot, _ = spatial_cdpr()
    state = RobotState([0.05, -0.04, 0.03, 0.08, -0.06, 0.05])
    result = solve_equilibrium_tensions(robot, state, reference_tension=20)
    assert result.feasible
    gravity = gravity_generalized_force(robot, state)
    assert np.linalg.norm(gravity[3:]) > 0.2
    np.testing.assert_allclose(result.residual, np.zeros(6), atol=1e-7, rtol=0)
    body = robot.bodies["moving_platform"]
    rotation = robot.frame_transform("moving_platform", state)[:3, :3]
    force = body.mass * robot.parameters.gravity
    wrench_g = np.r_[force, np.cross(rotation @ body.center_of_mass, force)]
    wrench = cable_wrench_matrix(robot, state, "moving_platform") @ result.tensions + wrench_g
    np.testing.assert_allclose(wrench, np.zeros(6), atol=2e-7, rtol=0)


def test_equal_baseline_and_translation_only_solve_leave_unbalanced_moments():
    robot, state = spatial_cdpr()
    matrix, gravity = robot.cable_force_matrix(state), gravity_generalized_force(robot, state)
    equal_residual = matrix @ np.full(8, 20.) + gravity
    assert np.linalg.norm(equal_residual) > 19
    assert np.linalg.norm(equal_residual[3:]) > 0.7
    lower, upper = robot.tension_bounds()
    partial = allocate_reference_tensions(matrix[:3], -gravity[:3], lower, upper, reference_tension=20)
    assert partial.feasible
    assert np.linalg.norm((matrix @ partial.tensions + gravity)[3:]) > 0.7
    full = solve_equilibrium_tensions(robot, state, reference_tension=20)
    assert full.feasible and full.residual_norm < 1e-7
    old = allocate_tensions(matrix, -gravity, lower, upper)
    assert old.feasible
    assert full.objective_value < 0.5 * np.linalg.norm(old.tensions - 20)**2 - 1


@pytest.mark.parametrize("factory", [serial_routed_mechanism, hybrid_mechanism])
def test_generic_external_and_on_robot_routes_with_known_feasible_load(factory):
    robot, state = factory()
    matrix = robot.cable_force_matrix(state)
    witness = np.linspace(10, 30, robot.cable_count)
    # The demonstration geometries need not support gravity alone. Supply a
    # known generalized applied load; don't claim physical untested feasibility.
    external = -matrix @ witness - gravity_generalized_force(robot, state)
    result = solve_equilibrium_tensions(robot, state, external, reference_tension=20)
    assert result.feasible, result.message
    np.testing.assert_allclose(result.residual, np.zeros(robot.dof), atol=1e-7, rtol=0)
    assert any(cable.name == "on_robot_routed" for cable in robot.cables)
    assert any(cable.name == "outer_to_tip" for cable in robot.cables)
    assert result.tensions.size == robot.cable_count
    if robot.dof == 9:
        np.testing.assert_allclose(matrix[:6, -2:], 0, atol=1e-8, rtol=0)


def test_local_translational_sweep_is_continuous_without_regularization():
    robot, initial = spatial_cdpr()
    tensions = []
    for x in np.linspace(-0.08, 0.08, 17):
        state = RobotState(initial.q + np.array([x, 0.3*x, -0.2*x, 0, 0, 0]))
        result = solve_equilibrium_tensions(robot, state, reference_tension=20)
        assert result.feasible, result.message
        assert result.residual_norm <= 1e-8 * (1 + np.linalg.norm(result.target_generalized_force))
        assert np.min(result.lower_margin) > 5
        tensions.append(result.tensions)
    assert np.max(np.abs(np.diff(tensions, axis=0))) < 1.0
