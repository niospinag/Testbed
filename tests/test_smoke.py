import numpy as np

from testbed import VirtualTestbed
from testbed.control.decentralized import DecentralizedController


def test_virtual_testbed_smoke():
    env = VirtualTestbed(number_of_robots=2, show_figure=False, sim_in_real_time=False)
    x0 = env.get_poses().copy()
    command = np.array([[50.0, 50.0], [0.0, 0.0]])
    env.set_velocities(np.arange(2), command)
    env.step()
    x1 = env.get_poses()
    assert x1.shape == (3, 2)
    assert np.any(np.abs(x1[:2, :] - x0[:2, :]) > 0)


def test_decentralized_controller_shape():
    n = 3
    poses = np.zeros((3, n))
    goals = np.zeros((3, n))

    def local_controller(robot_id, all_poses, all_goals):
        return np.array([float(robot_id), 0.5])

    decentralized = DecentralizedController(number_of_robots=n, controller_fn=local_controller)
    commands = decentralized.compute(poses, goals)
    assert commands.shape == (2, n)
    assert np.allclose(commands[1, :], 0.5)
