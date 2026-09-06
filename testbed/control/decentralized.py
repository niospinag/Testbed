from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


ControllerFn = Callable[[int, np.ndarray, np.ndarray], np.ndarray]


@dataclass
class DecentralizedController:
    """Compute one control command per robot in parallel threads."""

    number_of_robots: int
    controller_fn: ControllerFn
    max_workers: int | None = None

    def compute(self, poses: np.ndarray, goals: np.ndarray) -> np.ndarray:
        if poses.shape != (3, self.number_of_robots):
            raise ValueError(f"poses must be (3, {self.number_of_robots})")
        if goals.shape[1] != self.number_of_robots:
            raise ValueError(f"goals must have {self.number_of_robots} columns")

        def _for_robot(robot_id: int) -> np.ndarray:
            command = np.asarray(self.controller_fn(robot_id, poses, goals), dtype=float).reshape(-1)
            if command.shape[0] != 2:
                raise ValueError("controller_fn must return [v, w]")
            return command

        robot_ids: Iterable[int] = range(self.number_of_robots)
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            commands = list(executor.map(_for_robot, robot_ids))

        return np.asarray(commands, dtype=float).T
