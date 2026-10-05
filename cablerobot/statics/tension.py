"""Compatibility entry points for tension allocation."""

from cablerobot.allocation.tension import AllocationStatus, TensionAllocationResult, allocate_tensions, solve_tension_allocation

__all__ = ["AllocationStatus", "TensionAllocationResult", "allocate_tensions", "solve_tension_allocation"]

