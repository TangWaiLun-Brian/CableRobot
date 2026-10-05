import numpy as np
import pytest

from cablerobot.dynamics import forward_dynamics, inverse_dynamics


class SliderProvider:
    def mass_matrix(self, robot, state):
        return np.array([[2.]])

    def bias_force(self, robot, state):
        return np.array([19.62])


def test_numerical_dynamics_contract_and_gravity_sign(slider):
    robot, state = slider
    provider = SliderProvider()
    required = inverse_dynamics(robot, state, [1.], provider=provider)
    np.testing.assert_allclose(required, [21.62], atol=1e-8)
    acceleration = forward_dynamics(robot, state, required, provider=provider)
    np.testing.assert_allclose(acceleration, [1.], atol=1e-8)
    with_load = forward_dynamics(robot, state, required, external_load=[2.], provider=provider)
    np.testing.assert_allclose(with_load, [2.], atol=1e-8)


def test_invalid_mass_matrix_rejected(slider):
    robot, state = slider

    class InvalidProvider(SliderProvider):
        def mass_matrix(self, robot, state):
            return np.array([[-1.]])

    with pytest.raises(ValueError, match="positive definite"):
        inverse_dynamics(robot, state, [0.], provider=InvalidProvider())


@pytest.mark.parametrize("acceleration", [[1., 2.], [np.nan]])
def test_inverse_dynamics_validates_coordinates(slider, acceleration):
    robot, state = slider
    with pytest.raises(ValueError, match="qdd"):
        inverse_dynamics(robot, state, acceleration)
