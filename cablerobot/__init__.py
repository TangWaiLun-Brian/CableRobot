"""Generic cable-driven multibody robotics, with SI units and explicit signs."""

from .model import Attachment, Body, Cable, CableRobot, CableRoute, Frame, Joint, JointType, RobotParameters, RobotState, transform
from .allocation import AllocationStatus, TensionAllocationResult, allocate_tensions, solve_tension_allocation
from .allocation import EquilibriumTensionResult, allocate_reference_tensions, solve_equilibrium_tensions
from .kinematics import solve_configuration_from_lengths, solve_frame_position, solve_frame_pose
from .statics import cable_generalized_force, equilibrium_residual, gravity_generalized_force

__version__ = "0.1.0"

__all__ = [
    "AllocationStatus", "Attachment", "Body", "Cable", "CableRobot", "CableRoute",
    "Frame", "Joint", "JointType", "RobotParameters", "RobotState",
    "TensionAllocationResult", "allocate_tensions", "cable_generalized_force",
    "equilibrium_residual", "gravity_generalized_force", "solve_configuration_from_lengths",
    "solve_frame_position", "solve_tension_allocation", "transform",
    "solve_frame_pose",
    "EquilibriumTensionResult", "allocate_reference_tensions", "solve_equilibrium_tensions",
]
