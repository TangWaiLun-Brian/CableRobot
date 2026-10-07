"""Developer-only FK continuity experiment. Production solver/defaults unchanged."""

import argparse
import json
import os
from pathlib import Path
import platform
import sys
from time import perf_counter_ns

import numpy as np

from cablerobot import JointType, RobotState, solve_configuration_from_lengths
from cablerobot.examples import spatial_cdpr
from .control_pipeline import _git, timing_statistics, workload


def predict_initial_state(robot, previous, older=None, *, policy='previous'):
    if policy not in ('previous','constant_velocity'):
        raise ValueError('unknown warm-start policy')
    if policy=='previous' or older is None:
        return RobotState(previous.q.copy()),False
    predicted=2*previous.q-older.q
    # Extrapolation only in a conservative local floating chart. Return the whole
    # previous state on a branch/jump guard, not a partially extrapolated orientation.
    for joint in robot.joints:
        if joint.joint_type is JointType.FLOATING:
            block=slice(joint.q_start+3,joint.q_start+6)
            if (max(np.linalg.norm(q[block]) for q in (previous.q,older.q,predicted))>=np.pi-.1
                    or np.linalg.norm(previous.q[block]-older.q[block])>.1):
                return RobotState(previous.q.copy()),False
    if not np.all(np.isfinite(predicted)):
        return RobotState(previous.q.copy()),False
    return RobotState(predicted),True


def run_tracking(robot,qs,*,policy='previous',solver=solve_configuration_from_lengths):
    measured=[robot.cable_lengths(RobotState(q)) for q in qs]  # outside headline timing
    previous,older=RobotState(qs[0].copy()),None
    last_success=-1; rows=[]
    for index,lengths in enumerate(measured):
        start=perf_counter_ns()
        guess,predicted=predict_initial_state(robot,previous,older,policy=policy)
        result=solver(robot,lengths,guess)
        iterations=result.iterations; predictor_failure=predicted and not result.converged
        if predictor_failure:
            # Retrying accepted previous-state start counts both latency/iterations.
            retry=solver(robot,lengths,RobotState(previous.q.copy()))
            iterations+=retry.iterations; result=retry
        elapsed=(perf_counter_ns()-start)/1e6
        rows.append({'latency_ms':elapsed,'iterations':iterations,'status':result.status.value,
                     'length_residual':result.residual_norm,'predicted':predicted,
                     'predictor_failure':bool(predictor_failure),'initial_q':guess.q.tolist(),
                     'estimated_q':result.state.q.tolist(),
                     'coordinate_error':float(np.linalg.norm(result.state.q-qs[index]))})
        if result.converged:
            older=previous if last_success>=0 and last_success==index-1 else None
            previous=RobotState(result.state.q.copy()); last_success=index
        else:
            older=None  # last successful pose remains; consecutive-velocity history does not
    return {'policy':policy,'frames':len(rows),'failures':sum(r['status']!='converged' for r in rows),
            'predictor_failures':sum(r['predictor_failure'] for r in rows),
            'predicted_frames':sum(r['predicted'] for r in rows),
            'mean_iterations':float(np.mean([r['iterations'] for r in rows])),
            'p95_iterations':float(np.percentile([r['iterations'] for r in rows],95)),
            'total_iterations':sum(r['iterations'] for r in rows),
            'latency':timing_statistics([r['latency_ms'] for r in rows]),
            'max_length_residual':max(r['length_residual'] for r in rows),
            'max_coordinate_error':max(r['coordinate_error'] for r in rows),'rows':rows}


def run_comparison(output_dir,*,frames=500,repeats=3):
    if isinstance(repeats,bool) or not isinstance(repeats,int) or repeats<1:
        raise ValueError('repeats must be a positive integer')
    robot,qs,_=workload('repository_rotated',frames)
    _,practical,_=workload('practical_sample',frames)
    theta=2*np.pi*np.arange(frames)/frames
    moderate=qs.copy(); moderate[:,3:]=np.column_stack([.6*np.sin(theta),.4*np.cos(theta),.2*np.sin(theta/2)])
    branch=qs.copy(); branch[:,3:]=0; branch[:,5]=np.pi+.06*np.sin(theta)
    cases={'practical_translation':practical,'small_rotation':qs,'moderate_rotation':moderate,'near_pi_branch':branch}
    # All cases use the same accepted spatial model to isolate the start policy.
    report={'git_commit':_git(['rev-parse','HEAD']),'dirty_files':_git(['diff','--name-only']),
            'environment':{'python':sys.version,'numpy':np.__version__,'executable':sys.executable,
                           'platform':platform.platform(),'processor':platform.processor(),
                           'thread_environment':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','OMP_NUM_THREADS')}},
            'frames':frames,'repeats':repeats,
            'tolerance':1e-9,'first_guess':'first true pose, local tracking only',
            'timing_boundary':'predictor + FK incl any retry; lengths precomputed; no allocation/pacing/I/O',
            'scope':'developer experiment only; production previous-state default unchanged','cases':{}}
    for name,states in cases.items():
        measurements={'previous':[],'constant_velocity':[]}
        for repeat in range(repeats):
            # Alternate policy order to reduce systematic run-order bias.
            policies=('previous','constant_velocity') if repeat%2==0 else ('constant_velocity','previous')
            for policy in policies:
                solve_configuration_from_lengths(robot,robot.cable_lengths(RobotState(states[0])),RobotState(states[0]+.001))
                measurements[policy].append(run_tracking(robot,states,policy=policy))
        report['cases'][name]=measurements
        print(name,{p:{k:v for k,v in runs[0].items() if k!='rows'} for p,runs in measurements.items()},flush=True)
    output_dir=Path(output_dir); output_dir.mkdir(parents=True,exist_ok=True)
    (output_dir/'fk_prediction_results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames',type=int,default=500); parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--output-dir',type=Path,default=Path('examples/output/fk_prediction'))
    args=parser.parse_args(); run_comparison(args.output_dir,frames=args.frames,repeats=args.repeats)


if __name__=='__main__':
    main()
