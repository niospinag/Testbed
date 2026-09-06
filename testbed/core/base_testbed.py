from abc import ABC, abstractmethod
from typing import Dict

import numpy as np


class BaseTestbed(ABC):
    """Abstract base class shared by simulation and hardware backends."""

    def __init__(
        self,
        number_of_robots: int,
        show_figure: bool = True,
        initial_conditions: np.ndarray | None = None,
        config: dict | None = None,
    ):
        self._validate_inputs(number_of_robots, initial_conditions)
        self.config = config or {}
        self.number_of_robots = number_of_robots
        self.show_figure = show_figure

        robot_cfg = self.config.get("robot", {})
        arena_cfg = self.config.get("arena", {})
        self.time_step = float(robot_cfg.get("time_step", 0.033))
        self.robot_diameter = float(robot_cfg.get("diameter", 20.0))
        self.max_linear_velocity = float(robot_cfg.get("max_linear_velocity", 300.0))
        self.max_angular_velocity = float(robot_cfg.get("max_angular_velocity", 45.0))
        self.boundaries = self._parse_boundaries(
            arena_cfg.get("boundaries", [-200, 200, -150, 150])
        )

        self.poses = (
            initial_conditions
            if initial_conditions is not None
            else self._generate_initial_poses()
        )
        self.velocities = np.zeros((2, number_of_robots), dtype=float)
        self._iterations = 0
        self._errors: Dict[str, int] = {}
        self._called_step_already = True
        self._checked_poses_already = False

    @staticmethod
    def _parse_boundaries(boundaries):
        if len(boundaries) != 4:
            return (-200.0, 200.0, -150.0, 150.0)
        x0, x1, y0, y1 = [float(v) for v in boundaries]
        if x1 > x0 and y1 > y0:
            return (x0, x1, y0, y1)
        return (-200.0, 200.0, -150.0, 150.0)

    def _validate_inputs(self, number_of_robots, initial_conditions):
        assert isinstance(number_of_robots, int), "number_of_robots must be int"
        assert 0 < number_of_robots <= 50, "number_of_robots must be between 1 and 50"
        if initial_conditions is not None:
            assert isinstance(initial_conditions, np.ndarray), "initial_conditions must be np.ndarray"
            assert initial_conditions.shape == (
                3,
                number_of_robots,
            ), f"initial_conditions must be 3x{number_of_robots}"

    @abstractmethod
    def get_poses(self) -> np.ndarray:
        pass

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def _generate_initial_poses(self):
        pass

    def set_velocities(self, ids: np.ndarray, velocities: np.ndarray):
        assert isinstance(velocities, np.ndarray), "velocities must be np.ndarray"
        assert velocities.shape == (2, self.number_of_robots), (
            f"velocities must be shape (2, {self.number_of_robots})"
        )
        velocities = velocities.copy()
        velocities[0, :] = np.clip(
            velocities[0, :], -self.max_linear_velocity, self.max_linear_velocity
        )
        velocities[1, :] = np.clip(
            velocities[1, :], -self.max_angular_velocity, self.max_angular_velocity
        )
        self.velocities = velocities

    def validate_states(self) -> Dict[str, int]:
        errors: Dict[str, int] = {}
        xmin, xmax, ymin, ymax = self.boundaries
        for i in range(self.number_of_robots):
            x, y = self.poses[0, i], self.poses[1, i]
            if x < xmin or x > xmax or y < ymin or y > ymax:
                errors["boundary"] = errors.get("boundary", 0) + 1

        for i in range(self.number_of_robots - 1):
            for j in range(i + 1, self.number_of_robots):
                if np.linalg.norm(self.poses[:2, i] - self.poses[:2, j]) <= self.robot_diameter:
                    errors["collision"] = errors.get("collision", 0) + 1

        return errors

    def call_at_scripts_end(self):
        print("\n" + "=" * 50)
        print("TESTBED SUMMARY")
        print(f"Total iterations: {self._iterations}")
        if self._errors:
            print(f"State warnings: {self._errors}")
        print("=" * 50 + "\n")
