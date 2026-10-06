"""Independent derivative/physics/solver checks for the performance study."""

import numpy as np
import pytest

from cablerobot import (AllocationStatus, Attachment, Body, Cable, CableRobot, CableRoute,
                       Frame, Joint, RobotState, allocate_reference_tensions,
                       gravity_generalized_force, solve_configuration_from_lengths, transform)
from cablerobot.examples import spatial_cdpr, serial_mechanism, serial_routed_mechanism, hybrid_mechanism
from cablerobot.kinematics import SolverStatus, cable_jacobian, point_jacobian
from cablerobot.kinematics.bodies import point_position
from cablerobot.model.transforms import rotation_from_rotvec


def offset_forest():
    """Non-topological joint order, nested motion, rotated pre/post/frame offsets."""
    robot = CableRobot("Offset joint derivative forest")
    for name, fixed, mass in [("root",True,0), ("second_root",True,0), ("fixture",True,0),
                              ("carrier",False,1), ("slider",False,.4), ("tip",False,.2), ("tool",False,.1)]:
        robot.add_body(Body(name, fixed=fixed, mass=mass, center_of_mass=np.array([.07,-.03,.04])))
    def offset(translation, rotvec):
        return transform(rotation_from_rotvec(rotvec), translation)
    robot.add_joint(Joint("hinge", "slider", "tip", "revolute", axis=[.2,1,2],
                          T_parent_joint=offset([.3,.1,.02],[.1,-.2,.05]),
                          T_joint_child=offset([.04,.1,.03],[.1,.15,.2])))
    robot.add_joint(Joint("float", "fixture", "carrier", "floating",
                          T_parent_joint=offset([.1,.2,-.1],[.2,-.1,.15]),
                          T_joint_child=offset([.08,-.03,.05],[-.3,.1,0])))
    robot.add_joint(Joint("slide", "carrier", "slider", "prismatic", axis=[1,2,-1],
                          T_parent_joint=offset([.2,-.1,.04],[.15,.2,-.05]),
                          T_joint_child=offset([-.04,.08,.03],[.1,-.1,.02])))
    robot.add_joint(Joint("fixture_joint", "root", "fixture", "fixed",
                          T_parent_joint=offset([.1,.1,.2],[.1,.2,.3])))
    robot.add_joint(Joint("tool_joint", "tip", "tool", "fixed",
                          T_parent_joint=offset([.1,.04,.02],[.2,-.1,.2])))
    robot.add_frame(Frame("tip_sensor", "tip", offset([.03,-.1,.07],[.2,.1,-.2])))
    routes = [
        [Attachment("root",[1,1,1]), Attachment("carrier",[.2,.1,.05]),
         Attachment("tip_sensor",[.02,.03,.05]), Attachment("second_root",[-1,-.8,.9])],
        [Attachment("carrier",[.1,.2,.2]), Attachment("slider",[.2,-.2,.1]), Attachment("tool",[.2,.1,.08])],
        [Attachment("root",[.3,.8,.9]), Attachment("second_root",[-.4,.3,.2])],
        [Attachment("second_root",[-1,-1,-1]), Attachment("tip",[.2,-.2,-.2])],
    ]
    for index, points in enumerate(routes):
        robot.add_cable(Cable(f"route_{index}", CableRoute(points), tension_max=80.))
    return robot, RobotState([.25,.05,-.04,.03,.2,-.15,.1,.12])


def five_point(function, q, step=2e-5):
    reference = np.empty((np.asarray(function(q)).size, q.size))
    for column in range(q.size):
        delta = np.zeros(q.size)
        delta[column] = step
        reference[:, column] = (-function(q+2*delta)+8*function(q+delta)
                                -8*function(q-delta)+function(q-2*delta))/(12*step)
    return reference


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism, serial_routed_mechanism, hybrid_mechanism, offset_forest])
@pytest.mark.parametrize("seed", range(10))
def test_generic_random_routes_points_and_gravity_against_independent_derivatives(factory, seed):
    robot, initial = factory()
    rng = np.random.default_rng(600+seed)
    # 200 distinct translated/rotated/articulated configurations across the suite.
    for _ in range(4):
        state = RobotState(initial.q + rng.uniform(-.25,.25,robot.dof))
        reference = five_point(lambda q: robot.cable_lengths(RobotState(q)), state.q)
        analytic = robot.cable_jacobian(state)
        np.testing.assert_allclose(analytic, reference, atol=2e-8, rtol=2e-7)
        np.testing.assert_allclose(robot.cable_force_matrix(state), -analytic.T, atol=0, rtol=0)
        # Compare retained centered reference as well, not just another analytic path.
        np.testing.assert_allclose(analytic, cable_jacobian(robot,state,method="finite_difference"), atol=2e-8, rtol=2e-7)
        body = next(body for body in reversed(list(robot.bodies.values())) if not body.fixed)
        point = np.array([.12,-.08,.06])
        expected_point = five_point(lambda q: point_position(robot,body.name,point,RobotState(q)), state.q)
        np.testing.assert_allclose(point_jacobian(robot,state,body.name,point), expected_point, atol=2e-8, rtol=2e-7)
        gravity_fd = gravity_generalized_force(robot,state,jacobian_method="finite_difference")
        np.testing.assert_allclose(gravity_generalized_force(robot,state), gravity_fd, atol=2e-8, rtol=2e-7)
        velocity, tension = rng.normal(size=robot.dof), rng.uniform(1,10,robot.cable_count)
        h = 2e-5
        length_rate = (robot.cable_lengths(RobotState(state.q+h*velocity))
                       -robot.cable_lengths(RobotState(state.q-h*velocity)))/(2*h)
        assert (-analytic.T @ tension) @ velocity == pytest.approx(-tension @ length_rate, abs=2e-7, rel=2e-7)


@pytest.mark.parametrize("angle", [0.,1e-12,1e-5,.2,np.pi-1e-5,np.pi+.03,2*np.pi])
def test_rotvec_rate_map_not_cartesian_angular_velocity(angle):
    robot, _ = spatial_cdpr()
    axis = np.array([1.,2.,-1.])/np.sqrt(6)
    state = RobotState(np.r_[[.03,-.02,.01],angle*axis])
    expected = five_point(lambda q: robot.cable_lengths(RobotState(q)),state.q)
    np.testing.assert_allclose(robot.cable_jacobian(state), expected, atol=2e-8, rtol=2e-7)
    if angle == 2*np.pi:
        assert np.linalg.matrix_rank(robot.cable_jacobian(state),tol=1e-8) < 6


def test_step_preserves_reference_and_explicit_method_validation(slider):
    robot, state = slider
    np.testing.assert_allclose(cable_jacobian(robot,state,step=3e-6),
                               cable_jacobian(robot,state,step=3e-6,method="finite_difference"),atol=0,rtol=0)
    np.testing.assert_allclose(point_jacobian(robot,state,"carriage",np.zeros(3),step=3e-6), [[0],[0],[1]],atol=1e-9,rtol=0)
    with pytest.raises(ValueError,match="step"):
        cable_jacobian(robot,state,step=1e-6,method="analytic")
    for call in (lambda: robot.cable_jacobian(state,method="bad"),
                 lambda: solve_configuration_from_lengths(robot,[1.5],state,jacobian_method="bad"),
                 lambda: gravity_generalized_force(robot,state,jacobian_method="bad")):
        with pytest.raises(ValueError,match="method"):
            call()


@pytest.mark.parametrize("factory", [spatial_cdpr,serial_mechanism,serial_routed_mechanism,hybrid_mechanism])
def test_fk_analytic_and_numerical_recover_same_local_lengths(factory):
    robot, initial = factory()
    for offset in (.01,-.015):
        target = RobotState(initial.q+offset)
        lengths = robot.cable_lengths(target)
        guess = RobotState(initial.q-.01)
        analytic = solve_configuration_from_lengths(robot,lengths,guess)
        numeric = solve_configuration_from_lengths(robot,lengths,guess,jacobian_method="finite_difference")
        assert analytic.converged and numeric.converged
        assert analytic.residual_norm <= 1e-9 and numeric.residual_norm <= 1e-9
        np.testing.assert_allclose(analytic.state.q,numeric.state.q,atol=1e-7,rtol=0)
        np.testing.assert_allclose(analytic.state.q,target.q,atol=1e-6,rtol=0)


@pytest.mark.parametrize("offset", [0.,.01,-.02])
def test_tension_objective_equilibrium_and_infeasibility_are_preserved(offset):
    robot, initial = spatial_cdpr()
    state = RobotState(initial.q+offset)
    lower, upper = robot.tension_bounds()
    reference, weights = np.linspace(19,21,8), np.linspace(.8,1.3,8)
    analytic_matrix = robot.cable_force_matrix(state)
    numerical_matrix = -cable_jacobian(robot,state,method="finite_difference").T
    targets = [-gravity_generalized_force(robot,state),
               -gravity_generalized_force(robot,state,jacobian_method="finite_difference")]
    for bounds, status in [(upper,AllocationStatus.FEASIBLE),(np.full(8,2.),AllocationStatus.INFEASIBLE)]:
        analytic = allocate_reference_tensions(analytic_matrix,targets[0],lower,bounds,reference_tension=reference,weights=weights)
        numeric = allocate_reference_tensions(numerical_matrix,targets[1],lower,bounds,reference_tension=reference,weights=weights)
        assert analytic.status is status and numeric.status is status
        if analytic.feasible:
            np.testing.assert_allclose(analytic.tensions,numeric.tensions,atol=1e-6,rtol=0)
            assert analytic.objective_value == pytest.approx(numeric.objective_value,abs=1e-6,rel=1e-7)
            assert analytic.residual_norm < 1e-7 and numeric.residual_norm < 1e-7


def test_call_local_geometry_has_no_stale_mutation_cache(slider):
    robot,state = slider
    first = robot.cable_lengths(state).copy()
    robot.cables[0].route.points[0].point[0] = .3
    changed = robot.cable_lengths(state)
    assert not np.array_equal(first,changed)
    np.testing.assert_allclose(robot.cable_jacobian(state), five_point(lambda q: robot.cable_lengths(RobotState(q)),state.q),atol=1e-8,rtol=0)
    estimate = solve_configuration_from_lengths(robot,changed,RobotState([.4]))
    assert estimate.converged
    np.testing.assert_allclose(estimate.state.q,state.q,atol=1e-7,rtol=0)


def test_fk_failure_and_singular_geometry_are_not_disguised(monkeypatch):
    robot,state = spatial_cdpr()
    # Radial proportional attachments lose rotational observability at origin.
    for cable in robot.cables:
        cable.route.points[1].point = .1 * robot.frames[cable.route.points[0].frame].T_body_frame[:3,3]
    assert np.linalg.matrix_rank(robot.cable_jacobian(state),tol=1e-8) == 3
    result = solve_configuration_from_lengths(robot,robot.cable_lengths(state)+1, state,max_iterations=0)
    assert result.status is SolverStatus.MAX_ITERATIONS
    def failure(*args,**kwargs):
        raise np.linalg.LinAlgError("forced failure")
    monkeypatch.setattr(np.linalg,"solve",failure)
    result = solve_configuration_from_lengths(robot,robot.cable_lengths(state)+.01,state)
    assert result.status is SolverStatus.NUMERICAL_FAILURE and not result.converged
