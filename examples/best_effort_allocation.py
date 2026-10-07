"""Fixed gravity/bounds, prescribed poses: FEASIBLE -> BEST_EFFORT -> FEASIBLE."""

import argparse
from collections import Counter
import json
from pathlib import Path
from time import perf_counter_ns

import matplotlib.pyplot as plt
import numpy as np

from cablerobot import RobotState, solve_bounded_equilibrium_tensions, spatial_wrench_scaling
from cablerobot.analysis import frame_twist_jacobian
from cablerobot.examples import spatial_cdpr


def run_validation(output_dir, *, samples=61):
    if isinstance(samples,bool) or not isinstance(samples,int) or samples<5:
        raise ValueError('samples must be an integer >=5')
    robot,_=spatial_cdpr()
    for cable in robot.cables:
        cable.tension_max=30.
    rows=[]
    for time in np.linspace(0,6,samples):
        state=RobotState([1.2*np.sin(np.pi*time/6)**2,0,0,0,0,0])
        scaling=spatial_wrench_scaling(.2,generalized_force_from_wrench=
                                        frame_twist_jacobian(robot,state,'moving_platform').T)
        start=perf_counter_ns()
        result=solve_bounded_equilibrium_tensions(robot,state,residual_weights=scaling,reference_tension=20.)
        elapsed=(perf_counter_ns()-start)/1e6
        if result.tension_command is None:
            raise RuntimeError(f'unverified trajectory proposal at {time}: {result.message}')
        rows.append({'time_s':float(time),'q':state.q.tolist(),'status':result.status.value,
                     'exact_equilibrium':result.exact_equilibrium,'tensions_N':result.tensions.tolist(),
                     'command_available':result.tension_command is not None,
                     'B':robot.cable_force_matrix(state).tolist(),
                     'target':result.target_generalized_force.tolist(),
                     'scaling_matrix':result.residual_scaling.matrix.tolist(),
                     'physical_residual':result.residual.tolist(),
                     'force_residual_N':result.force_residual.tolist(),
                     'moment_residual_Nm':result.moment_residual.tolist(),
                     'weighted_residual_norm':result.weighted_residual_norm,
                     'unweighted_residual_norm':result.residual_norm,
                     'objective':result.objective_value,'reference_objective':result.reference_objective_value,
                     'active_lower':result.active_lower.tolist(),'active_upper':result.active_upper.tolist(),
                     'primary_duality_gap':result.primary_duality_gap,'primary_kkt_violation':result.primary_kkt_violation,
                     'force_image_change_norm':result.force_image_change_norm,'latency_ms':elapsed})
    counts=dict(Counter(r['status'] for r in rows))
    transitions=[rows[0]['status']]+[b['status'] for a,b in zip(rows,rows[1:]) if a['status']!=b['status']]
    if transitions!=['feasible','best_effort','feasible']:
        raise RuntimeError(f'expected a real feasibility-boundary crossing, got {transitions}')
    report={'model':robot.name,'samples':samples,'duration_s':6.,'characteristic_length_m':.2,
            'bounds_N':[1.,30.],'reference_tension_N':20.,'status_counts':counts,'transitions':transitions,
            'max_weighted_residual':max(r['weighted_residual_norm'] for r in rows),
            'max_primary_duality_gap':max((r['primary_duality_gap'] or 0) for r in rows),
            'max_force_image_change':max((r['force_image_change_norm'] or 0) for r in rows),
            'max_neighbor_command_change_N':max(float(np.max(np.abs(np.array(b['tensions_N'])-a['tensions_N']))) for a,b in zip(rows,rows[1:])),
            'interpretation':'Prescribed static poses, not simulated physical motion. BEST_EFFORT requires other support -residual or motion may occur.',
            'cable_names':[c.name for c in robot.cables],'rows':rows}
    output_dir=Path(output_dir); output_dir.mkdir(parents=True,exist_ok=True)
    (output_dir/'best_effort_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    _plot(report,output_dir/'best_effort_trajectory.png')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','cable_names')},indent=2))
    return report


def _plot(report,path):
    rows=report['rows']; time=[r['time_s'] for r in rows]
    fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    tensions=np.array([r['tensions_N'] for r in rows])
    for i,name in enumerate(report['cable_names']):
        axes[0,0].plot(time,tensions[:,i],label=name)
    axes[0,0].axhline(1,color='black',ls=':'); axes[0,0].axhline(30,color='black',ls=':')
    axes[0,0].set(title='Bounded tension proposals',ylabel='Tension [N]'); axes[0,0].legend(ncols=2,fontsize=8)
    axes[0,1].step(time,[r['status']=='feasible' for r in rows],where='mid')
    axes[0,1].set(title='Exact vs best effort',yticks=[0,1],yticklabels=['BEST_EFFORT','FEASIBLE'],ylim=(-.2,1.2))
    axes[1,0].semilogy(time,np.maximum([r['weighted_residual_norm'] for r in rows],1e-14))
    axes[1,0].set(title='Explicitly scaled residual',ylabel='Force-equivalent norm [N]')
    for i,label in enumerate(('x','y','z')):
        axes[1,1].plot(time,np.array([r['force_residual_N'] for r in rows])[:,i],label=label)
        axes[2,0].plot(time,np.array([r['moment_residual_Nm'] for r in rows])[:,i],label=label)
    axes[1,1].set(title='Uncompensated world force',ylabel='Force [N]')
    axes[2,0].set(title='Uncompensated moment about platform origin',ylabel='Moment [N m]')
    axes[1,1].legend(); axes[2,0].legend()
    active=np.array([np.array(r['active_lower'],int)+2*np.array(r['active_upper'],int) for r in rows]).T
    axes[2,1].imshow(active,origin='lower',aspect='auto',interpolation='nearest',extent=[0,6,.5,8.5],vmin=0,vmax=2,cmap='viridis')
    axes[2,1].set(title='Active bounds: dark=free, mid=lower, light=upper',ylabel='Cable index')
    for ax in axes.flat:
        ax.set_xlabel('Prescribed path time [s]'); ax.grid(alpha=.2)
    fig.suptitle('Static proposals across a true boundary — no hardware or dynamic-motion claim')
    fig.savefig(path,dpi=130); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=Path('examples/output/best_effort'))
    parser.add_argument('--samples',type=int,default=61)
    args=parser.parse_args(); run_validation(args.output_dir,samples=args.samples)


if __name__=='__main__':
    main()
