import matplotlib.pyplot as plt
import pytest

from cablerobot.examples import serial_mechanism, spatial_cdpr
from cablerobot.visualization import plot_robot


@pytest.mark.parametrize("factory", [spatial_cdpr, serial_mechanism])
def test_topology_independent_visualization(factory, tmp_path):
    robot, state = factory()
    ax = plot_robot(robot, state, selected_frames=list(robot.bodies), tensions=[1.] * robot.cable_count)
    assert len(ax.lines) >= robot.cable_count + len(robot.joints)
    path = tmp_path / "robot.png"
    ax.figure.savefig(path, dpi=60)
    assert path.stat().st_size > 1000
    plt.close(ax.figure)
