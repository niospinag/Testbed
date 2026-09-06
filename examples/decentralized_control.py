"""Decentralized threaded control example for multiple robots."""

import numpy as np

from testbed import VirtualTestbed
from testbed.control.controllers import create_pid_unicycle_pose_controller
from testbed.control.decentralized import DecentralizedController
from testbed.utils.geometry import at_pose


def main():
    n = 5
    env = VirtualTestbed(number_of_robots=n, show_figure=True)
    pid = create_pid_unicycle_pose_controller(
        linear_gain=[8, 0.0, 0.0],
        angular_gain=[14, 0.1, 0.2],
        num_robots=n,
    )

    goals = np.array(
        [
            [120.0, 70.0, 0.0, -70.0, -120.0],
            [0.0, 90.0, 120.0, 90.0, 0.0],
            [0.0, 0.0, np.pi / 2, np.pi, -np.pi / 2],
        ]
    )

    def local_controller(robot_id: int, poses: np.ndarray, target_poses: np.ndarray) -> np.ndarray:
        # Reuse global controller but expose per-robot command for decentralized threading.
        all_commands = pid(poses, target_poses)
        return all_commands[:, robot_id]

    decentralized = DecentralizedController(number_of_robots=n, controller_fn=local_controller)

    _ = env.get_poses()
    env.step()
    env.draw_point(goals)

    for _ in range(800):
        x = env.get_poses()
        if np.size(at_pose(x, goals, position_error=12, rotation_error=0.3)) == n:
            break
        u = decentralized.compute(x, goals)
        env.set_velocities(np.arange(n), u)
        env.step()

    env.call_at_scripts_end()


if __name__ == "__main__":
    main()
