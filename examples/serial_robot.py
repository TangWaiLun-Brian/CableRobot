"""Run with python examples/serial_robot.py --output examples/output/serial.png."""

import argparse
from pathlib import Path

import numpy as np

from cablerobot import RobotState, cable_generalized_force
from cablerobot.examples import serial_mechanism
from cablerobot.visualization import plot_robot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    robot, state = serial_mechanism()
    print("body transforms:")
    for name, T in robot.body_transforms(state).items():
        print(name, np.round(T, 4))
    print("lengths [m]:", np.round(robot.cable_lengths(state), 6))
    print("perturbed lengths [m]:", np.round(robot.cable_lengths(RobotState(state.q + [0.1, 0.1])), 6))
    print("cable Jacobian:", np.round(robot.cable_jacobian(state), 6))
    print("generalized cable force [N m]:", np.round(cable_generalized_force(robot, state, [5., 3., 2., 4.]), 6))
    ax = plot_robot(robot, state, selected_frames=["ground", "link_1", "link_2", "tip"])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ax.figure.savefig(args.output, dpi=150, bbox_inches="tight")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
