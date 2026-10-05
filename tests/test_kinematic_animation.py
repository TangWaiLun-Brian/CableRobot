import matplotlib.pyplot as plt
import numpy as np
import pytest
from PIL import Image

from cablerobot.examples import routing_animation_panels, serial_routed_mechanism
from cablerobot.kinematics import cable_route_points
from cablerobot.simulation import KinematicTrajectory, sinusoidal_trajectory
from cablerobot.visualization import animate_robots


def test_sampled_trajectory_initial_state_derivative_and_duration():
    robot, initial = serial_routed_mechanism()
    amplitude = np.array([0.2, 0.3, 0.4])
    phase = np.array([0., 0.3, 0.6])
    trajectory = sinusoidal_trajectory(robot, initial, amplitude, duration=5., fps=20., phases=phase)
    assert len(trajectory.times) == 100 and trajectory.duration == 5.
    assert trajectory.times[-1] == pytest.approx(4.95, abs=1e-12)
    np.testing.assert_allclose(trajectory.q[0], initial.q, atol=1e-12)
    omega = 2 * np.pi / 5
    h = 1e-6
    at_time = lambda t: initial.q + amplitude * (np.sin(omega * t + phase) - np.sin(phase))
    reference = (at_time(1.5 + h) - at_time(1.5 - h)) / (2 * h)
    np.testing.assert_allclose(trajectory.qd[30], reference, atol=1e-9)
    np.testing.assert_allclose(at_time(5.), initial.q, atol=1e-12)


@pytest.mark.parametrize("name", ["spatial", "serial", "hybrid"])
def test_all_animated_routes_remain_valid_and_change_over_motion(name):
    panel = routing_animation_panels(duration=1., fps=12.)[name]
    values = []
    for i in range(len(panel.trajectory.times)):
        state = panel.trajectory.state_at(i)
        values.append(panel.robot.cable_lengths(state))
        for cable in panel.robot.cables:
            route = cable_route_points(panel.robot, cable, state)
            assert np.all(np.linalg.norm(np.diff(route, axis=0), axis=1) > 1e-3)
    assert np.all(np.ptp(values, axis=0) > 0.01)


def test_rendered_artists_follow_full_routes_and_saved_gif_timing(tmp_path):
    panels = routing_animation_panels(duration=0.3, fps=10.)
    rendered = animate_robots(list(panels.values()), figsize=(9, 3.6))
    try:
        rendered.draw_frame(2)
        for panel, artists in zip(panels.values(), rendered.cable_artists):
            for cable in panel.robot.cables:
                displayed = np.vstack(artists[cable.name].get_data_3d()).T
                expected = cable_route_points(panel.robot, cable, panel.trajectory.state_at(2))
                np.testing.assert_allclose(displayed, expected, atol=1e-12)
        path = tmp_path / "three_robots.gif"
        rendered.save(path, dpi=50)
        with Image.open(path) as gif:
            assert gif.n_frames == 3
            assert gif.info["loop"] == 0
            duration = 0
            for index in range(gif.n_frames):
                gif.seek(index)
                duration += gif.info["duration"]
            assert duration == 300
    finally:
        rendered.close()
    assert not plt.fignum_exists(rendered.figure.number)


@pytest.mark.parametrize("kwargs", [{"duration": -1.}, {"fps": 0.}, {"duration": 0.33, "fps": 10.}])
def test_trajectory_parameters_rejected(kwargs):
    robot, initial = serial_routed_mechanism()
    with pytest.raises(ValueError):
        sinusoidal_trajectory(robot, initial, [0.2, 0.3, 0.4], **kwargs)


def test_mismatched_state_shape_and_nonuniform_time_rejected():
    robot, initial = serial_routed_mechanism()
    with pytest.raises(ValueError, match="amplitudes"):
        sinusoidal_trajectory(robot, initial, [0.2])
    with pytest.raises(ValueError, match="uniformly"):
        KinematicTrajectory(np.array([0., 0.11]), np.zeros((2, 3)), np.zeros((2, 3)), 10.)
