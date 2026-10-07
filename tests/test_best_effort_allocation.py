"""Independent primal-face/KKT oracles; accepted exact tests remain untouched."""

from itertools import product

import numpy as np
import pytest

from cablerobot import (BoundedAllocationStatus as Status, ResidualScaling, RobotState,
                       allocate_best_effort_tensions, solve_bounded_equilibrium_tensions,
                       solve_equilibrium_tensions, gravity_generalized_force, spatial_wrench_scaling)
from cablerobot.analysis import frame_twist_jacobian, cable_wrench_matrix
from cablerobot.examples import spatial_cdpr, serial_routed_mechanism, hybrid_mechanism
import cablerobot.allocation.best_effort as implementation


def _oracle(a, b, lower, upper, reference, weights):
    """Independent augmented primal KKT systems, not production dual algorithms."""
    minimum, image = np.inf, None
    states = list(product((0, 1, 2), repeat=len(lower)))
    for state in states:
        free = np.array(state) == 0
        t = np.where(np.array(state) == 1, lower, upper).copy()
        af = a[:, free]
        rhs = b-a[:, ~free] @ t[~free]
        block = np.block([[np.eye(len(b)), af], [af.T, np.zeros((sum(free), sum(free)))]])
        solution = np.linalg.lstsq(block, np.r_[rhs, np.zeros(sum(free))], rcond=None)[0]
        t[free] = solution[len(b):]
        if np.any(t < lower-1e-9) or np.any(t > upper+1e-9):
            continue
        objective = .5*np.linalg.norm(a@t-b)**2
        if objective < minimum:
            minimum, image = objective, a@t
    secondary = np.inf
    for state in states:
        free = np.array(state) == 0
        t = np.where(np.array(state) == 1, lower, upper).copy()
        af = a[:,free]
        h = np.diag(weights[free]**2)
        block = np.block([[h, af.T], [af, np.zeros((len(b), len(b)))]])
        rhs = np.r_[weights[free]**2*reference[free], image-a[:,~free]@t[~free]]
        solution = np.linalg.lstsq(block, rhs, rcond=None)[0]
        t[free] = solution[:sum(free)]
        if (np.any(t < lower-1e-8) or np.any(t > upper+1e-8)
                or np.linalg.norm(a@t-image) > 1e-7):
            continue
        secondary = min(secondary, .5*np.linalg.norm(weights*(t-reference))**2)
    return minimum, secondary


@pytest.mark.parametrize("matrix,target,lower,upper,expected,residual", [
    ([[1]], [12], [0], [10], [10], [-2]),
    ([[1]], [0], [5], [10], [5], [5]),
    ([[1],[0]], [4,3], [0], [10], [4], [0,-3]),
    ([[1],[2]], [0,5], [0], [10], [2], [2,-1]),
])
def test_analytical_infeasible_primary_optimum(matrix,target,lower,upper,expected,residual):
    result = allocate_best_effort_tensions(matrix,target,lower,upper,residual_weights=1,reference_tension=3)
    assert result.status is Status.BEST_EFFORT and not result.feasible and not result.exact_equilibrium
    assert result.primary_optimality_verified and result.secondary_optimality_verified
    np.testing.assert_allclose(result.tensions,expected,atol=1e-9,rtol=0)
    np.testing.assert_allclose(result.residual,residual,atol=1e-9,rtol=0)
    command = result.tension_command
    command[:] = 0
    np.testing.assert_allclose(result.tensions,expected,atol=1e-9,rtol=0)


def test_lexicographic_redundancy_reference_cannot_buy_worse_residual():
    result = allocate_best_effort_tensions([[1,1],[0,0]],[6,2],[0,0],[10,10],
                                           residual_weights=[1,5],reference_tension=[1,3],weights=[1,2])
    np.testing.assert_allclose(result.tensions,[2.6,3.4],atol=1e-8,rtol=0)
    np.testing.assert_allclose(result.residual,[0,-2],atol=1e-10,rtol=0)
    assert result.weighted_residual_norm == pytest.approx(10)
    assert result.force_residual is None and result.moment_residual is None
    huge = allocate_best_effort_tensions([[1]],[12],[0],[10],residual_weights=1,reference_tension=1e6)
    assert huge.status is Status.BEST_EFFORT
    np.testing.assert_allclose(huge.tensions,[10],atol=1e-9,rtol=0)


@pytest.mark.parametrize("seed",range(25))
def test_independent_primary_and_secondary_oracle(seed):
    rng = np.random.default_rng(seed+403)
    matrix = rng.normal(size=(3,4))
    lower, upper = np.full(4,.5), np.full(4,4.)
    target = rng.normal(size=3)*15
    scale = rng.uniform(.2,4,3)
    reference, weights = rng.uniform(0,10,4), rng.uniform(.5,3,4)
    result = allocate_best_effort_tensions(matrix,target,lower,upper,residual_weights=scale,
                                           reference_tension=reference,weights=weights)
    assert result.status in (Status.FEASIBLE,Status.BEST_EFFORT),result.message
    primary, secondary = _oracle(scale[:,None]*matrix,scale*target,lower,upper,reference,weights)
    assert result.objective_value == pytest.approx(primary,abs=3e-7,rel=1e-9)
    assert result.reference_objective_value == pytest.approx(secondary,abs=3e-6,rel=1e-8)
    assert np.all(result.tensions>=lower) and np.all(result.tensions<=upper)
    np.testing.assert_allclose(result.residual,matrix@result.tensions-target,atol=1e-13,rtol=0)
    # Independent random bounded competitors cannot improve the primary optimum.
    candidates = rng.uniform(lower,upper,size=(100,4))
    assert np.min(np.linalg.norm((matrix@candidates.T-target[:,None])*scale[:,None],axis=0)) >= result.weighted_residual_norm-1e-7


def test_weighted_and_dense_scaling_are_real_objectives_not_metadata():
    unweighted = allocate_best_effort_tensions([[1],[2]],[0,5],[0],[10],residual_weights=1)
    weighted = allocate_best_effort_tensions([[1],[2]],[0,5],[0],[10],residual_weights=[1,10])
    np.testing.assert_allclose(unweighted.tensions,[2],atol=1e-9)
    np.testing.assert_allclose(weighted.tensions,[1000/401],atol=1e-9)
    dense = ResidualScaling([[1,1],[0,2]])
    result = allocate_best_effort_tensions([[1],[2]],[0,5],[0],[10],residual_weights=dense)
    expected = np.linalg.lstsq(dense.matrix@np.array([[1.],[2.]]),dense.matrix@[0,5],rcond=None)[0]
    np.testing.assert_allclose(result.tensions,expected,atol=1e-9)


def test_mixed_wrench_characteristic_length_and_rotated_covectors():
    robot,_ = spatial_cdpr()
    state = RobotState([.03,-.02,.01,.4,-.3,.2])
    for c in robot.cables:
        c.tension_max=2
    scaling = spatial_wrench_scaling(.2,generalized_force_from_wrench=frame_twist_jacobian(robot,state,'moving_platform').T)
    result = solve_bounded_equilibrium_tensions(robot,state,residual_weights=scaling,reference_tension=20)
    assert result.status is Status.BEST_EFFORT,result.message
    body = robot.bodies['moving_platform']
    force = body.mass*robot.parameters.gravity
    com = robot.frame_transform('moving_platform',state)[:3,:3]@body.center_of_mass
    physical = cable_wrench_matrix(robot,state,'moving_platform')@result.tensions+np.r_[force,np.cross(com,force)]
    np.testing.assert_allclose(np.r_[result.force_residual,result.moment_residual],physical,atol=2e-7,rtol=0)
    assert result.weighted_residual_norm == pytest.approx(np.linalg.norm(np.r_[physical[:3],physical[3:]/.2]),abs=2e-7)
    assert np.linalg.norm(result.residual[3:]-result.moment_residual)>1e-3


@pytest.mark.parametrize("factory",[serial_routed_mechanism,hybrid_mechanism])
def test_generic_routes_under_deliberately_impossible_load(factory):
    robot,state = factory()
    matrix = robot.cable_force_matrix(state)
    # A row target beyond its independently computed box support.
    lower,upper = robot.tension_bounds()
    target = matrix@np.linspace(10,30,robot.cable_count)
    target[0] = np.maximum(matrix[0],0)@upper+np.minimum(matrix[0],0)@lower+100
    external = -target-gravity_generalized_force(robot,state)
    result = solve_bounded_equilibrium_tensions(robot,state,external,residual_weights=np.ones(robot.dof),reference_tension=20)
    assert result.status is Status.BEST_EFFORT,result.message
    np.testing.assert_allclose(result.residual,matrix@result.tensions-target,atol=1e-9,rtol=0)
    assert result.residual.shape == (robot.dof,) and result.force_residual is None
    assert any(c.name=='on_robot_routed' for c in robot.cables)
    assert any(c.name=='outer_to_tip' for c in robot.cables)


def test_exact_first_is_identical_and_does_not_run_both(monkeypatch):
    robot,state=spatial_cdpr()
    exact=solve_equilibrium_tensions(robot,state,reference_tension=20,weights=np.arange(1,9))
    def forbidden(*args,**kwargs):
        raise AssertionError('fallback must not run on a verified exact solution')
    monkeypatch.setattr(implementation,'_allocate_best_effort_tensions',forbidden)
    monkeypatch.setattr(implementation,'_primary_candidate',forbidden)
    result=solve_bounded_equilibrium_tensions(robot,state,residual_weights=[1,1,1,5,5,5],reference_tension=20,weights=np.arange(1,9))
    assert result.status is Status.FEASIBLE and result.exact_equilibrium
    np.testing.assert_array_equal(result.tensions,exact.tensions)
    np.testing.assert_array_equal(result.residual,exact.residual)
    assert result.reference_objective_value==exact.objective_value
    assert result.exact_result.duality_gap==exact.duality_gap


def test_exact_only_preserves_infeasible_no_command(slider):
    robot,state=slider
    result=solve_bounded_equilibrium_tensions(robot,state,[-100],residual_weights=1,best_effort=False)
    assert result.status is Status.INFEASIBLE and result.tension_command is None
    assert result.exact_result.status.value=='infeasible'


def test_certified_impossible_screen_skips_exact_qp_and_reuses_primary(slider,monkeypatch):
    robot,state=slider
    original_primary=implementation._primary_candidate
    original_reference=implementation.allocate_reference_tensions
    primary_calls=[]; reference_targets=[]
    def primary(*args):
        primary_calls.append(1)
        return original_primary(*args)
    def reference(matrix,target,*args,**kwargs):
        reference_targets.append(target.copy())
        return original_reference(matrix,target,*args,**kwargs)
    monkeypatch.setattr(implementation,'_primary_candidate',primary)
    monkeypatch.setattr(implementation,'allocate_reference_tensions',reference)
    result=solve_bounded_equilibrium_tensions(robot,state,[-100],residual_weights=1)
    assert result.status is Status.BEST_EFFORT and result.exact_result is None
    assert len(primary_calls)==1 and len(reference_targets)==1
    np.testing.assert_allclose(reference_targets[0],result.achieved_generalized_force,atol=1e-12)
    assert not np.allclose(reference_targets[0],result.target_generalized_force)


def test_exact_only_never_screens_primary(slider,monkeypatch):
    def forbidden(*args,**kwargs):
        raise AssertionError('exact-only cannot call the new primary screen')
    monkeypatch.setattr(implementation,'_primary_candidate',forbidden)
    robot,state=slider
    result=solve_bounded_equilibrium_tensions(robot,state,residual_weights=1,best_effort=False)
    assert result.status is Status.FEASIBLE


def test_inconclusive_projected_candidate_uses_small_reference_fallback(monkeypatch):
    monkeypatch.setattr(implementation,'_projected_least_squares',lambda *args: np.array([0.]))
    result=allocate_best_effort_tensions([[1]],[12],[0],[10],residual_weights=1)
    assert result.status is Status.BEST_EFFORT
    np.testing.assert_array_equal(result.tensions,[10.])


@pytest.mark.parametrize("matrix,target,lower,upper,expected",[
    (np.zeros((0,2)),[],[1,1],[10,10],[3,3]),
    (np.zeros((2,0)),[2,0],[],[],[]),
    (np.zeros((2,2)),[2,0],[1,1],[10,10],[3,3]),
    ([[1,0],[0,0]],[2,1],[0,0],[np.inf,np.inf],[2,3]),
    ([[1,1],[0,0]],[7,1],[2,0],[2,np.inf],[2,5]),
])
def test_empty_fixed_infinite_and_zero_mapping(matrix,target,lower,upper,expected):
    result=allocate_best_effort_tensions(matrix,target,lower,upper,residual_weights=1,reference_tension=3)
    assert result.status in (Status.FEASIBLE,Status.BEST_EFFORT),result.message
    np.testing.assert_allclose(result.tensions,expected,atol=1e-8,rtol=0)


@pytest.mark.parametrize("scale",[1e-4,1,1e4])
def test_force_scales_and_near_infeasibility(scale):
    result=allocate_best_effort_tensions([[scale]],[12*scale],[0],[10],residual_weights=1,reference_tension=3)
    assert result.status is Status.BEST_EFFORT,result.message
    np.testing.assert_allclose(result.tensions,[10],atol=1e-8,rtol=0)
    near=allocate_best_effort_tensions([[1]],[10+1e-5],[0],[10],residual_weights=1,reference_tension=3)
    assert near.status is Status.BEST_EFFORT and 9e-6<near.residual_norm<11e-6


def test_iteration_failure_nonfinite_and_unverified_candidates_are_not_commands(monkeypatch):
    exhausted=allocate_best_effort_tensions([[1,1,0],[0,1,1]],[1,2],[0]*3,[20]*3,
                                            residual_weights=1,reference_tension=10,max_iterations=1)
    assert exhausted.status is Status.NUMERICALLY_UNRESOLVED and exhausted.tension_command is None
    def fail(*args,**kwargs):
        raise np.linalg.LinAlgError('forced least-squares failure')
    monkeypatch.setattr(np.linalg,'lstsq',fail)
    failed=allocate_best_effort_tensions([[1]],[12],[0],[10],residual_weights=1)
    assert failed.status is Status.NUMERICAL_FAILURE and failed.tension_command is None
    monkeypatch.undo()
    overflow=allocate_best_effort_tensions([[1e200]],[1e200],[0],[10],residual_weights=1)
    assert overflow.status is Status.NUMERICAL_FAILURE and overflow.tension_command is None


def test_poorly_conditioned_nine_cable_witness_never_becomes_false_best_effort():
    matrix=np.zeros((2,9)); matrix[0,:3]=[1,1,-1]; matrix[1,1]=1e-5
    result=allocate_best_effort_tensions(matrix,[0,1],np.zeros(9),np.full(9,1e6),residual_weights=1)
    assert result.status in (Status.FEASIBLE,Status.NUMERICALLY_UNRESOLVED)
    if result.status is Status.NUMERICALLY_UNRESOLVED:
        assert result.tension_command is None


def test_roundoff_at_active_bound_uses_projected_kkt_not_exact_float_equality():
    verified,gap,kkt=implementation._primary_certificate(np.array([[1.]]),np.array([12.]),
                                                         np.array([0.]),np.array([10.]),
                                                         np.array([10.-1e-12]),1e-8)
    assert verified and gap<1e-8 and kkt<1e-8
    bad,_,_=implementation._primary_certificate(np.array([[1.]]),np.array([12.]),
                                                np.array([0.]),np.array([10.]),np.array([9.]),1e-8)
    assert not bad


def test_small_gradient_gap_cannot_certify_false_best_effort(monkeypatch):
    from cablerobot.allocation.tension import TensionAllocationResult, AllocationStatus
    def inaccurate(matrix,target,lower,upper,**kwargs):
        return TensionAllocationResult(AllocationStatus.NUMERICALLY_UNRESOLVED,np.array([0.]),
                                       np.array([0.]),target,-target,float(np.linalg.norm(target)),
                                       'forced inaccurate candidate with tiny objective gap')
    monkeypatch.setattr(implementation,'allocate_tensions',inaccurate)
    monkeypatch.setattr(implementation,'_projected_least_squares',
                        lambda *args: np.array([0.]))
    # Feasible witness t=.5 exists. Coarse objective-gap tolerance alone would
    # accept t=0; physical separation must prevent a false BEST_EFFORT command.
    result=allocate_best_effort_tensions([[1e-5]],[5e-6],[0],[1],residual_weights=1)
    assert result.status is Status.NUMERICALLY_UNRESOLVED
    assert not result.infeasibility_certified and result.tension_command is None


def test_infinite_upper_support_is_not_capped_or_ignored():
    verified,gap,_=implementation._primary_certificate(np.array([[1.]]),np.array([1e-12]),
                                                       np.array([0.]),np.array([np.inf]),np.array([0.]),1e-8)
    assert not verified and gap is None


@pytest.mark.parametrize("kwargs",[
    {'residual_weights':None},{'residual_weights':0},{'residual_weights':-1},
    {'residual_weights':np.inf},{'residual_weights':[1,2]},{'residual_weights':[[1]]},
    {'reference_tension':-1},{'reference_tension':np.inf},{'weights':0},
    {'optimality_tolerance':0},{'tolerance':np.nan},{'max_iterations':0},{'max_iterations':True},
])
def test_invalid_options(kwargs):
    options={'residual_weights':1}; options.update(kwargs)
    with pytest.raises(ValueError):
        allocate_best_effort_tensions([[1]],[1],[0],[10],**options)


@pytest.mark.parametrize("matrix,target,lower,upper",[
    ([1],[1],[0],[10]),([[1]],[1,2],[0],[10]),([[1]],[1],[0,0],[10]),
    ([[np.nan]],[1],[0],[10]),([[1]],[np.inf],[0],[10]),([[1]],[1],[-1],[10]),
    ([[1]],[1],[2],[1]),([[1]],[1],[0],[np.nan]),([[1]],[1],[np.inf],[np.inf]),
])
def test_invalid_problem(matrix,target,lower,upper):
    with pytest.raises(ValueError):
        allocate_best_effort_tensions(matrix,target,lower,upper,residual_weights=1)


@pytest.mark.parametrize("matrix",[[[1,1],[1,1]],[[1,0],[0,1e-14]],[[np.nan]],[[1,2]]])
def test_invalid_scaling_matrix(matrix):
    with pytest.raises(ValueError):
        ResidualScaling(matrix)


@pytest.mark.parametrize("length",[0,-1,np.nan,np.inf])
def test_invalid_characteristic_length(length):
    with pytest.raises(ValueError):
        spatial_wrench_scaling(length)
