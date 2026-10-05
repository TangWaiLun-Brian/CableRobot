import numpy as np
import pytest

from cablerobot import AllocationStatus, RobotState, allocate_tensions, cable_generalized_force, equilibrium_residual, gravity_generalized_force, solve_tension_allocation
from cablerobot.analysis import cable_wrench_matrix, frame_twist_jacobian
from cablerobot.examples import serial_mechanism, spatial_cdpr


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_virtual_work_by_directional_length_change(factory):
    robot, state = factory()
    velocity = np.linspace(0.2, -0.1, robot.dof)
    tensions = np.arange(1., robot.cable_count + 1.)
    h = 2e-5
    rate = (robot.cable_lengths(RobotState(state.q + h * velocity)) - robot.cable_lengths(RobotState(state.q - h * velocity))) / (2 * h)
    power = float(cable_generalized_force(robot, state, tensions) @ velocity)
    assert power == pytest.approx(-tensions @ rate, abs=1e-7)


def test_gravity_prismatic(slider):
    robot, state = slider
    np.testing.assert_allclose(gravity_generalized_force(robot, state), [-19.62], atol=1e-8)


def test_gravity_revolute_with_offset_com(pendulum):
    robot, state = pendulum
    expected = 3. * 9.81 * 0.4 * np.cos(state.q[0])
    np.testing.assert_allclose(gravity_generalized_force(robot, state), [expected], atol=1e-8)


def test_floating_gravity_includes_moment_at_offset_com():
    robot, state = spatial_cdpr()
    g = 2. * 9.81
    np.testing.assert_allclose(gravity_generalized_force(robot, state), [0, 0, -g, 0.02 * g, 0.03 * g, 0], atol=2e-8)


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_body_wrenches_recover_generalized_force(factory):
    robot, state = factory()
    state = RobotState(state.q + 0.13)
    mapping = np.zeros((robot.dof, robot.cable_count))
    for name in robot.bodies:
        mapping += frame_twist_jacobian(robot, state, name).T @ cable_wrench_matrix(robot, state, name)
    np.testing.assert_allclose(mapping, robot.cable_force_matrix(state), atol=2e-8)


def test_feasible_slider_equilibrium_and_bounds(slider):
    robot, state = slider
    result = solve_tension_allocation(robot, state)
    assert result.status is AllocationStatus.FEASIBLE
    np.testing.assert_allclose(result.tensions, [19.62], atol=1e-7)
    np.testing.assert_allclose(equilibrium_residual(robot, state, result.tensions), [0], atol=1e-7)
    lower, upper = robot.tension_bounds()
    assert np.all(result.tensions >= lower) and np.all(result.tensions <= upper)


def test_infeasible_slider_equilibrium(slider):
    robot, state = slider
    robot.cables[0].tension_max = 10.
    result = solve_tension_allocation(robot, state)
    assert result.status is AllocationStatus.INFEASIBLE
    np.testing.assert_allclose(result.tensions, [10.], atol=1e-9)
    assert result.residual_norm == pytest.approx(9.62, abs=1e-7)


def test_external_load_is_added_to_gravity(slider):
    robot, state = slider
    result = solve_tension_allocation(robot, state, external_load=[-5.])
    np.testing.assert_allclose(result.tensions, [24.62], atol=1e-7)
    np.testing.assert_allclose(equilibrium_residual(robot, state, result.tensions, [-5.]), [0], atol=1e-7)


def test_cdpr_gravity_allocation_and_overload():
    robot, state = spatial_cdpr()
    feasible = solve_tension_allocation(robot, state)
    assert feasible.feasible
    assert feasible.residual_norm < 1e-6
    infeasible = solve_tension_allocation(robot, state, [0, 0, -10000, 0, 0, 0])
    assert infeasible.status is AllocationStatus.INFEASIBLE


def test_active_bound_solution():
    result = allocate_tensions([[1., -1.]], [4.], [2., 2.], [6., 5.])
    assert result.feasible
    np.testing.assert_allclose([[1., -1.]] @ result.tensions, [4.], atol=1e-9)
    assert result.tensions[0] <= 6. and result.tensions[1] >= 2.


def test_unbounded_upper_tension():
    result = allocate_tensions([[1.]], [1000.], [0.], [np.inf])
    assert result.feasible
    np.testing.assert_allclose(result.tensions, [1000.], atol=1e-9)


@pytest.mark.parametrize("target,status", [(18., AllocationStatus.FEASIBLE), (100., AllocationStatus.INFEASIBLE)])
def test_large_reference_allocation(target, status):
    result = allocate_tensions(np.ones((1, 9)), [target], np.zeros(9), np.full(9, 3.))
    assert result.status is status


def test_ill_conditioned_allocation_reports_unresolved():
    B = np.zeros((2, 9))
    B[0, :3] = [1., 1., -1.]
    B[1, 1] = 1e-5
    result = allocate_tensions(B, [0., 1.], np.zeros(9), np.full(9, 1e6))
    # This one is feasible, but the reference solver cannot certify it in its iteration budget.
    # With no false infeasibility certificate, the caller can choose another numerical backend.
    assert result.status is AllocationStatus.NUMERICALLY_UNRESOLVED


@pytest.mark.parametrize("lower,upper", [([-1.], [2.]), ([3.], [2.]), ([np.nan], [1.]), ([0.], [np.nan])])
def test_invalid_allocation_bounds(lower, upper):
    with pytest.raises(ValueError):
        allocate_tensions([[1.]], [1.], lower, upper)


def test_route_reversal_preserves_cable_forces():
    from cablerobot import CableRoute

    robot, state = serial_mechanism()
    original = robot.cable_force_matrix(state)
    wrench = cable_wrench_matrix(robot, state, "tip")
    lengths = robot.cable_lengths(state)
    for cable in robot.cables:
        cable.route = CableRoute(list(reversed(cable.route.points)))
    np.testing.assert_allclose(robot.cable_lengths(state), lengths, atol=1e-12)
    np.testing.assert_allclose(robot.cable_force_matrix(state), original, atol=1e-8)
    np.testing.assert_allclose(cable_wrench_matrix(robot, state, "tip"), wrench, atol=1e-12)


@pytest.mark.parametrize("target,status", [([0.], AllocationStatus.FEASIBLE), ([1.], AllocationStatus.INFEASIBLE)])
def test_no_cable_allocation(target, status):
    result = allocate_tensions(np.zeros((1, 0)), target, [], [])
    assert result.status is status
    assert result.tensions.shape == (0,)


def test_numerical_failure_does_not_claim_infeasibility(monkeypatch):
    def failed_lstsq(*args, **kwargs):
        raise np.linalg.LinAlgError("test linear algebra failure")

    monkeypatch.setattr(np.linalg, "lstsq", failed_lstsq)
    result = allocate_tensions([[1.]], [3.], [1.], [5.])
    assert result.status is AllocationStatus.NUMERICAL_FAILURE
    np.testing.assert_allclose(result.achieved_generalized_force, [1.], atol=1e-12)
