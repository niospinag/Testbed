import os
from typing import Dict, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np


class VisionSystem:
    """ArUco-based robot pose tracking."""

    def __init__(
        self,
        camera_id: int = 0,
        resolution: Tuple[int, int] = (1024, 576),
        marker_size: float = 10.2,
        camera_config_path: str = "config/camera",
    ):
        self.camera_id = camera_id
        self.width, self.height = resolution
        self.marker_size = marker_size
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        self.camera_matrix, self.camera_distortion = self._load_calibration(camera_config_path)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()

    def _load_calibration(self, path: str) -> Tuple[np.ndarray, np.ndarray]:
        matrix_path = os.path.join(path, "cameraMatrix.txt")
        distortion_path = os.path.join(path, "cameraDistortion.txt")
        matrix = np.loadtxt(matrix_path, delimiter=",")
        distortion = np.loadtxt(distortion_path, delimiter=",")
        matrix[0, 2] = self.width / 2
        matrix[1, 2] = self.height / 2
        matrix[1, 1] = -matrix[1, 1]
        return matrix, distortion

    @staticmethod
    def _calculate_orientation(corner: np.ndarray) -> float:
        p1 = corner[0, 0]
        p4 = corner[0, 3]
        return np.arctan2(p4[1] - p1[1], p1[0] - p4[0])

    def detect_robots(self) -> Tuple[Dict[int, np.ndarray], Optional[np.ndarray]]:
        success, frame = self.cap.read()
        if not success:
            raise RuntimeError("Failed to capture frame from camera")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        poses: Dict[int, np.ndarray] = {}
        if ids is not None:
            _, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.camera_distortion
            )
            for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
                marker_id = int(marker_id)
                theta = self._calculate_orientation(corner)
                poses[marker_id] = np.array([tvecs[i, 0, 0], tvecs[i, 0, 1], theta], dtype=float)
                aruco.drawDetectedMarkers(frame, [corner])
                top_left = tuple(corner[0, 0].astype(int))
                cv2.putText(frame, str(marker_id), top_left, cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
        return poses, frame

    def draw_goals(self, frame: np.ndarray, goals: np.ndarray):
        factor = int(14 * (self.height / self.width)) / self.marker_size
        for i in range(goals.shape[1]):
            goal_px = (goals[:2, i] * [1, -1] * factor * 3 + [self.width / 2, self.height / 2]).astype(int)
            cv2.circle(frame, tuple(goal_px), 7, (0, 200, 0), -1)

    def release(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
