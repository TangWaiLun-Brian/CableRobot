"""Study scripts tested without installing developer modules in the wheel."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from cablerobot import RobotState
from cablerobot.examples import spatial_cdpr
from cablerobot.kinematics.inverse import KinematicSolverResult, SolverStatus


def _script(name,path):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).resolve().parents[1]/path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


_demo=_script('best_effort_demo','examples/best_effort_allocation.py')
# Relative benchmark import uses a package-qualified name without changing sys.path.
_prediction=_script('benchmarks.fk_prediction','benchmarks/fk_prediction.py')


def test_demo_crosses_real_boundary_with_bounded_commands(tmp_path):
    report=_demo.run_validation(tmp_path,samples=9)
    assert report['transitions']==['feasible','best_effort','feasible']
    for row in report['rows']:
        tensions=np.array(row['tensions_N'])
        assert np.all(tensions>=1) and np.all(tensions<=30) and row['command_available']
        np.testing.assert_allclose(np.array(row['B'])@tensions-row['target'],row['physical_residual'],atol=1e-9,rtol=0)
    assert (tmp_path/'best_effort_report.json').exists() and (tmp_path/'best_effort_trajectory.png').exists()


@pytest.mark.parametrize('angle',[np.pi-.05,np.pi+.01,2*np.pi-.01])
def test_prediction_falls_back_near_chart_limits(angle):
    robot,_=spatial_cdpr()
    previous=RobotState([.01,0,0,0,0,angle]); older=RobotState([0,0,0,0,0,angle-.01])
    guess,used=_prediction.predict_initial_state(robot,previous,older,policy='constant_velocity')
    assert not used; np.testing.assert_array_equal(guess.q,previous.q)


def test_small_rotation_prediction_and_representation_jump_guard():
    robot,_=spatial_cdpr()
    previous=RobotState([.01,0,0,0,0,.2]); older=RobotState([0,0,0,0,0,.19])
    guess,used=_prediction.predict_initial_state(robot,previous,older,policy='constant_velocity')
    assert used; np.testing.assert_allclose(guess.q,[.02,0,0,0,0,.21],atol=1e-15)
    previous.q[5]=-3.13; older.q[5]=3.13
    guess,used=_prediction.predict_initial_state(robot,previous,older,policy='constant_velocity')
    assert not used; np.testing.assert_array_equal(guess.q,previous.q)


@pytest.mark.parametrize('policy',['previous','constant_velocity'])
def test_only_successful_history_is_used_and_failure_resets_velocity(policy):
    robot,_=spatial_cdpr(); guesses=[]
    qs=np.zeros((5,6)); qs[:,0]=np.arange(5)*.001
    calls=0
    def solver(robot,lengths,guess):
        nonlocal calls
        guesses.append(guess.q.copy()); i=calls; calls+=1
        # A final failure in both policies before a predictor is eligible.
        if i==1:
            return KinematicSolverResult(SolverStatus.MAX_ITERATIONS,RobotState(np.ones(6)),1.,0,'forced failure')
        return KinematicSolverResult(SolverStatus.CONVERGED,RobotState(qs[i]),0.,0,'test success')
    result=_prediction.run_tracking(robot,qs,policy=policy,solver=solver)
    assert result['failures']==1
    np.testing.assert_array_equal(guesses[1],qs[0]); np.testing.assert_array_equal(guesses[2],qs[0])
    np.testing.assert_array_equal(guesses[3],qs[2])


def test_real_tracking_with_previous_and_predictor():
    robot,qs,_=_prediction.workload('repository_rotated',500)
    for policy in ('previous','constant_velocity'):
        result=_prediction.run_tracking(robot,qs[:8],policy=policy)
        assert result['failures']==0 and result['max_length_residual']<=1e-9
        assert result['max_coordinate_error']<1e-6
        if policy=='previous':
            for a,b in zip(result['rows'],result['rows'][1:]):
                np.testing.assert_array_equal(b['initial_q'],a['estimated_q'])


def test_failed_prediction_retries_previous_and_counts_cost():
    robot,_=spatial_cdpr(); qs=np.zeros((4,6)); qs[:,0]=np.arange(4)*.001
    calls=[]
    def solver(robot,lengths,guess):
        i=len(calls); calls.append(guess.q.copy())
        if i==2:
            return KinematicSolverResult(SolverStatus.MAX_ITERATIONS,RobotState(np.ones(6)),1.,5,'forced predictor failure')
        frame=i if i<2 else i-1
        return KinematicSolverResult(SolverStatus.CONVERGED,RobotState(qs[frame]),0.,2,'test success')
    result=_prediction.run_tracking(robot,qs,policy='constant_velocity',solver=solver)
    assert result['failures']==0 and result['predictor_failures']==1
    assert result['rows'][2]['iterations']==7
    np.testing.assert_array_equal(calls[3],qs[1])


def test_invalid_study_options(tmp_path):
    with pytest.raises(ValueError): _demo.run_validation(tmp_path,samples=4)
    with pytest.raises(ValueError): _prediction.run_comparison(tmp_path,frames=5,repeats=0)
    robot,state=spatial_cdpr()
    with pytest.raises(ValueError): _prediction.predict_initial_state(robot,state,policy='fixed_reset')


@pytest.mark.parametrize('failures',[0,1])
def test_cli_convergence_smoke_flag_enforces_recorded_result(failures,monkeypatch,tmp_path):
    import sys
    monkeypatch.setattr(sys,'argv',['fk_prediction','--frames','64','--repeats','1',
                                    '--require-convergence','--output-dir',str(tmp_path)])
    monkeypatch.setattr(_prediction,'run_comparison',lambda *args,**kwargs:
                        {'cases':{'case':{'previous':[{'failures':failures}]}}})
    if failures:
        with pytest.raises(RuntimeError,match='convergence failures'):
            _prediction.main()
    else:
        _prediction.main()
