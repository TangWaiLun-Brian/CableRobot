import matplotlib
matplotlib.use("Agg")

import numpy as np
import pytest

from cablerobot import Attachment, Body, Cable, CableRobot, CableRoute, Joint, RobotState


@pytest.fixture
def slider():
    robot = CableRobot("one-axis slider")
    robot.add_body(Body("earth", fixed=True))
    robot.add_body(Body("carriage", mass=2.0))
    robot.add_joint(Joint("vertical", "earth", "carriage", "prismatic", axis=[0, 0, 1]))
    robot.add_cable(Cable("lift", CableRoute([
        Attachment("earth", np.array([0., 0., 2.])), Attachment("carriage", np.zeros(3)),
    ]), tension_max=50.))
    return robot, RobotState([0.5])


@pytest.fixture
def pendulum():
    robot = CableRobot("offset center-of-mass pendulum")
    robot.add_body(Body("mount", fixed=True))
    robot.add_body(Body("arm", mass=3.0, center_of_mass=np.array([0.4, 0.0, 0.0])))
    robot.add_joint(Joint("pivot", "mount", "arm", "revolute", axis=[0, 1, 0]))
    return robot, RobotState([0.3])
