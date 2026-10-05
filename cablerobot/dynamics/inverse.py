"""Inverse-dynamics interface and implemented static special case."""

from __future__ import annotations

from cablerobot.statics.gravity import gravity_generalized_force


def static_generalized_force_required(robot, state):
    """Return the non-gravity generalized force required to hold the state."""
    return -gravity_generalized_force(robot, state)


def inverse_dynamics(robot, state, qdd, *, provider=None):
    """Return required total generalized force M qdd + h, before cable allocation."""
    from .equations import FullDynamicsDeferredError, numerical_terms, validate_generalized_vector

    acceleration = validate_generalized_vector(robot, state, qdd, "qdd")
    if provider is not None:
        matrix, bias = numerical_terms(robot, state, provider)
        return matrix @ acceleration + bias

    raise FullDynamicsDeferredError("accelerating inverse dynamics is deferred; use static_generalized_force_required for milestone 1")
