"""Basic closed-loop control example in virtual simulation."""

import numpy as np

from testbed import VirtualTestbed
from testbed.control.controllers import create_pid_unicycle_pose_controller
from testbed.utils.geometry import at_pose


def main():
    num_robots = 3
    env = VirtualTestbed(number_of_robots=num_robots, show_figure=True)
    controller = create_pid_unicycle_pose_controller(
        linear_gain=[6, 0.0, 0.0],
        angular_gain=[12, 0.1, 0.2],
        num_robots=num_robots,
    )

    goals = np.array(
        [
            [80.0, -80.0, 0.0],
            [60.0, 60.0, -60.0],
            [0.0, np.pi / 2, -np.pi / 2],
        ]
    )

    x = env.get_poses()
    env.step()
    env.draw_point(goals)

    for _ in range(1000):
        x = env.get_poses()
        if np.size(at_pose(x, goals, position_error=10, rotation_error=0.2)) == num_robots:
            break
        u = controller(x, goals)
        env.set_velocities(np.arange(num_robots), u)
        env.step()

    env.call_at_scripts_end()


if __name__ == "__main__":
    main()
