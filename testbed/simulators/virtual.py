import time
from typing import Optional

import numpy as np

from testbed.core.base_testbed import BaseTestbed
from testbed.simulators.plotlab import Plotlab
from testbed.utils import geometry


class VirtualTestbed(BaseTestbed):
    """Virtual simulator for differential-drive robots."""

    def __init__(
        self,
        number_of_robots: int = 1,
        show_figure: bool = True,
        sim_in_real_time: bool = True,
        initial_conditions: Optional[np.ndarray] = None,
        config: Optional[dict] = None,
    ):
        self.sim_in_real_time = sim_in_real_time
        self._last_step_time = time.time()
        super().__init__(number_of_robots, show_figure, initial_conditions, config)
        self.visual = None
        if self.show_figure:
            self.visual = Plotlab(
                number_of_robots=self.number_of_robots,
                show_figure=True,
                initial_conditions=self.poses,
            )

    def _generate_initial_poses(self):
        return geometry.generate_initial_conditions(
            self.number_of_robots, spacing=30, width=200, height=150
        )

    def get_poses(self) -> np.ndarray:
        assert not self._checked_poses_already, "Can only call get_poses() once per step()."
        self._called_step_already = False
        self._checked_poses_already = True
        return self.poses

    def step(self):
        assert not self._called_step_already, "Must call get_poses() before step()."
        self._called_step_already = True
        self._checked_poses_already = False
        self._errors.update(self.validate_states())
        self._iterations += 1

        self.poses[0, :] = self.poses[0, :] + self.time_step * np.cos(self.poses[2, :]) * self.velocities[0, :]
        self.poses[1, :] = self.poses[1, :] + self.time_step * np.sin(self.poses[2, :]) * self.velocities[0, :]
        self.poses[2, :] = self.poses[2, :] + self.time_step * self.velocities[1, :]
        self.poses[2, :] = np.arctan2(np.sin(self.poses[2, :]), np.cos(self.poses[2, :]))

        if self.visual is not None:
            self.visual.step(self.poses)

        if self.sim_in_real_time:
            elapsed = time.time() - self._last_step_time
            remaining = self.time_step - elapsed
            if remaining > 0:
                time.sleep(remaining)
            self._last_step_time = time.time()

    def draw_point(self, goals: np.ndarray):
        if self.visual is not None:
            self.visual.draw_point(goals)
