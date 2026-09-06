"""
Real hardware testbed implementation with vision tracking and serial communication.
"""

import time
from typing import Optional

import cv2
import numpy as np

import testbed.control.controllers as ctrl
import testbed.utils.geometry as geometry
from testbed.config.settings import TestbedConfig
from testbed.core.base_testbed import BaseTestbed
from testbed.core.robot import Robot, RobotParameters
from testbed.hardware.communication import MultiSerialCommunicator, SerialCommunicator
from testbed.hardware.vision import VisionSystem


class RealTestbed(BaseTestbed):
    """Real hardware backend."""

    def __init__(
        self,
        number_of_robots: int,
        show_figure: bool = True,
        initial_conditions: Optional[np.ndarray] = None,
        config: Optional[TestbedConfig] = None,
        move_to_initial: bool = True,
    ):
        self.config_obj = config or TestbedConfig()
        super().__init__(
            number_of_robots,
            show_figure,
            initial_conditions,
            self.config_obj.to_dict(),
        )
        self.vision = VisionSystem(
            camera_id=self.config_obj.vision.camera_id,
            resolution=self.config_obj.vision.resolution,
            marker_size=self.config_obj.vision.marker_size,
            camera_config_path=self.config_obj.vision.camera_config_path,
        )
        self.communicator = self._init_communication()
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.recording = False
        self.goals: Optional[np.ndarray] = None
        self.show_goals = False
        self.robots = [
            Robot(i, self.poses[:, i], RobotParameters(**self.config_obj.robot.__dict__))
            for i in range(number_of_robots)
        ]
        if initial_conditions is not None and move_to_initial:
            self._move_to_initial_conditions(initial_conditions)
        else:
            self.poses = self._detect_initial_poses()

    def _generate_initial_poses(self) -> np.ndarray:
        return np.zeros((3, self.number_of_robots))

    def _init_communication(self):
        comm_cfg = self.config_obj.communication
        if self.number_of_robots <= comm_cfg.robots_per_antenna:
            return SerialCommunicator(port=comm_cfg.primary_port, baudrate=comm_cfg.baudrate)
        ports = [comm_cfg.primary_port, comm_cfg.secondary_port]
        return MultiSerialCommunicator(ports, comm_cfg.baudrate)

    def _detect_initial_poses(self) -> np.ndarray:
        max_attempts = 10
        poses_dict = {}
        for _ in range(max_attempts):
            poses_dict, _ = self.vision.detect_robots()
            if len(poses_dict) >= self.number_of_robots:
                poses = np.zeros((3, self.number_of_robots))
                for i in range(self.number_of_robots):
                    if (i + 1) in poses_dict:
                        poses[:, i] = poses_dict[i + 1]
                return poses
            time.sleep(0.5)
        raise RuntimeError(f"Could not detect {self.number_of_robots} robots. Only found {len(poses_dict)}")

    def _move_to_initial_conditions(self, target_poses: np.ndarray):
        controller = ctrl.create_reactive_pose_controller(
            linear_gain=[9, 0.1, 0],
            angular_gain=[14, 0.1, 1],
            num_robots=self.number_of_robots,
        )
        max_iterations = 400
        position_tol = 10.0
        rotation_tol = 0.2
        for _ in range(max_iterations):
            x = self.get_poses()
            at_target = geometry.at_pose(x, target_poses, position_tol, rotation_tol)
            if np.size(at_target) == self.number_of_robots:
                break
            dxu = controller(x, target_poses)
            self.set_velocities(np.arange(self.number_of_robots), dxu)
            self.step()
        self.show_goals = False
        time.sleep(0.5)

    def get_poses(self) -> np.ndarray:
        assert not self._checked_poses_already, "Can only call get_poses() once per step()"
        self._called_step_already = False
        self._checked_poses_already = True
        poses_dict, frame = self.vision.detect_robots()
        for robot_id, pose in poses_dict.items():
            if 1 <= robot_id <= self.number_of_robots:
                self.poses[:, robot_id - 1] = pose
                self.robots[robot_id - 1].update_pose(pose)
        if self.show_figure:
            self._visualize(frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.call_at_scripts_end()
        return self.poses

    def _visualize(self, frame: np.ndarray):
        self._draw_axes(frame)
        if self.show_goals and self.goals is not None:
            self.vision.draw_goals(frame, self.goals)
        cv2.imshow("Testbed - Real", frame)
        if self.recording and self.video_writer:
            self.video_writer.write(frame)

    @staticmethod
    def _draw_axes(frame: np.ndarray):
        h, w = frame.shape[:2]
        color = (100, 100, 100)
        cv2.line(frame, (w // 2, 0), (w // 2, h), color, 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), color, 1)

    def step(self):
        assert not self._called_step_already, "Must call get_poses() before step()"
        self._called_step_already = True
        self._checked_poses_already = False
        self._errors.update(self.validate_states())
        self._iterations += 1
        robot_ids = np.arange(self.number_of_robots)
        self.communicator.send_velocities(robot_ids, self.velocities)

    def set_goals(self, goals: np.ndarray):
        assert isinstance(goals, np.ndarray), f"goals must be np.ndarray, got {type(goals)}"
        self.goals = goals
        self.show_goals = True
        for i in range(self.number_of_robots):
            goal = goals[:, i] if goals.shape[0] == 3 else np.append(goals[:, i], 0)
            self.robots[i].set_goal(goal)

    def start_recording(self, filename: str):
        import os

        os.makedirs("Videos", exist_ok=True)
        fps = int(self.vision.cap.get(cv2.CAP_PROP_FPS)) or 30
        output_path = f"Videos/{filename}.avi"
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.video_writer = cv2.VideoWriter(output_path, fourcc, fps, (self.vision.width, self.vision.height))
        self.recording = True

    def stop_recording(self):
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        self.recording = False

    def call_at_scripts_end(self):
        super().call_at_scripts_end()
        if self.recording:
            self.stop_recording()
        self.communicator.close()
        self.vision.release()
