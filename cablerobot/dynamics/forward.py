"""Forward-dynamics interface."""


def forward_dynamics(robot, state, tensions, external_load=None, *, provider=None):
    """Solve supplied numerical dynamics terms; default full model dynamics is deferred.

    provider.bias_force must include gravity with the h = -tau_gravity convention.
    external_load contains applied loads additional to that gravity.
    """
    import numpy as np

    from .equations import FullDynamicsDeferredError, numerical_terms, validate_generalized_vector

    load = validate_generalized_vector(robot, state, np.zeros(robot.dof) if external_load is None else external_load, "external_load")
    tension = np.asarray(tensions, dtype=float)
    if tension.shape != (robot.cable_count,) or not np.all(np.isfinite(tension)) or np.any(tension < 0.0):
        raise ValueError("tensions must be a finite, nonnegative cable vector")
    if provider is not None:
        matrix, bias = numerical_terms(robot, state, provider)
        return np.linalg.solve(matrix, robot.cable_force_matrix(state) @ tension + load - bias)

    raise FullDynamicsDeferredError("forward dynamics is deferred beyond milestone 1")
