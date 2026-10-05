"""Run with python examples/spatial_cdpr.py --output examples/output/cdpr.png."""

import argparse
from pathlib import Path

import numpy as np

from cablerobot import equilibrium_residual, gravity_generalized_force, solve_tension_allocation
from cablerobot.examples import spatial_cdpr
from cablerobot.visualization import plot_robot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    robot, state = spatial_cdpr()
    result = solve_tension_allocation(robot, state)
    print("lengths [m]:", np.round(robot.cable_lengths(state), 6))
    print("cable Jacobian shape/rank:", robot.cable_jacobian(state).shape, np.linalg.matrix_rank(robot.cable_jacobian(state)))
    print("gravity generalized force:", np.round(gravity_generalized_force(robot, state), 6))
    print("allocation:", result.status.value, "tensions [N]:", np.round(result.tensions, 4))
    print("equilibrium residual norm:", np.linalg.norm(equilibrium_residual(robot, state, result.tensions)))
    assert result.feasible, result.message
    impossible = solve_tension_allocation(robot, state, external_load=np.array([0., 0., -10000., 0., 0., 0.]))
    print("overload allocation:", impossible.status.value)
    assert impossible.status.value == "infeasible"
    ax = plot_robot(robot, state, tensions=result.tensions, selected_frames=["outer_frame", "moving_platform"])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ax.figure.savefig(args.output, dpi=150, bbox_inches="tight")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
