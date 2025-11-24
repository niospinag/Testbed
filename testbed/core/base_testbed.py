from abc import ABC, abstractmethod
import numpy as np

class BaseTestbed(ABC):
    """Abstract base class for testbed implementations."""
    
    def __init__(self, number_of_robots: int, show_figure: bool = True, 
                 initial_conditions: np.ndarray = None, config: dict = None):
        self._validate_inputs(number_of_robots, initial_conditions)
        
        self.number_of_robots = number_of_robots
        self.show_figure = show_figure
        # Importamos aquí dentro para evitar problemas circulares o usamos config pasado
        self.config = config 
        
        # Parámetros por defecto si no hay config (Hardcoded por seguridad)
        self.robot_diameter = 20.0
        self.max_linear_velocity = 300.0
        self.max_angular_velocity = 45.0
        self.time_step = 0.033

        # State variables
        self.poses = initial_conditions if initial_conditions is not None else \
                     self._generate_initial_poses()
        self.velocities = np.zeros((2, number_of_robots))
        
        # Tracking
        self._iterations = 0
        self._errors = {}
        self._called_step_already = True
        self._checked_poses_already = False
    
    def _validate_inputs(self, number_of_robots, initial_conditions):
        assert isinstance(number_of_robots, int), "number_of_robots must be int"
        assert 0 < number_of_robots <= 30, "number_of_robots must be between 1-30"
        if initial_conditions is not None:
            assert isinstance(initial_conditions, np.ndarray), "initial_conditions must be np.ndarray"

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
        # Saturate linear velocities
        idxs = np.where(np.abs(velocities[0, :]) > self.max_linear_velocity)
        velocities[0, idxs] = self.max_linear_velocity * np.sign(velocities[0, idxs])
        # Saturate angular velocities
        idxs = np.where(np.abs(velocities[1, :]) > self.max_angular_velocity)
        velocities[1, idxs] = self.max_angular_velocity * np.sign(velocities[1, idxs])
        self.velocities = velocities
    
    def call_at_scripts_end(self):
        print('\n' + '='*50)
        print('TESTBED SUMMARY')
        print(f'Total iterations: {self._iterations}')
        print('='*50 + '\n')



"""
Vision system for robot tracking using ArUco markers.
"""
import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Optional, Tuple, Dict
import os


class VisionSystem:
    """ArUco-based vision tracking system."""
    
    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (1024, 576),
                 marker_size: float = 10.2, camera_config_path: str = "Camera"):
        """
        Initialize vision system.
        
        Args:
            camera_id: Camera device ID
            resolution: (width, height) in pixels
            marker_size: ArUco marker size in cm
            camera_config_path: Path to camera calibration files
        """
        self.camera_id = camera_id
        self.width, self.height = resolution
        self.marker_size = marker_size
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        
        # Load camera calibration
        self.camera_matrix, self.camera_distortion = self._load_calibration(
            camera_config_path
        )
        
        # ArUco setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Load marker augmentation images (optional)
        self.marker_images = self._load_marker_images("Markers")
        
        print(f"✓ Vision system initialized (Camera {camera_id})")
    
    def _load_calibration(self, path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Load camera calibration matrices."""
        matrix = np.loadtxt(f'{path}/cameraMatrix.txt', delimiter=',')
        distortion = np.loadtxt(f'{path}/cameraDistortion.txt', delimiter=',')
        
        # Adjust for resolution
        matrix[0, 2] = self.width / 2
        matrix[1, 2] = self.height / 2
        matrix[1, 1] = -matrix[1, 1]
        
        return matrix, distortion
    
    def _load_marker_images(self, path: str) -> Dict[int, np.ndarray]:
        """Load marker augmentation images."""
        if not os.path.exists(path):
            return {}
        
        marker_imgs = {}
        for img_file in os.listdir(path):
            marker_id = int(os.path.splitext(img_file)[0])
            img = cv2.imread(f'{path}/{img_file}')
            marker_imgs[marker_id] = img
        
        return marker_imgs
    
    def detect_robots(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Detect robots and return their poses.
        
        Returns:
            poses: 3xN array of [x, y, theta] poses
            frame: Annotated camera frame (if visualization enabled)
        """
        success, frame = self.cap.read()
        if not success:
            raise RuntimeError("Failed to capture frame from camera")
        
        # Detect ArUco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        
        poses = {}
        
        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, 
                self.camera_matrix, self.camera_distortion
            )
            
            for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
                # Calculate orientation from marker corners
                theta = self._calculate_orientation(corner)
                
                # Store pose
                poses[marker_id] = np.array([
                    tvecs[i, 0, 0],  # x
                    tvecs[i, 0, 1],  # y
                    theta             # orientation
                ])
                
                # Draw on frame
                aruco.drawDetectedMarkers(frame, [corner])
                self._draw_marker_id(frame, corner, marker_id)
        
        return poses, frame
    
    def _calculate_orientation(self, corner: np.ndarray) -> float:
        """Calculate marker orientation from corners."""
        p1 = corner[0, 0]  # First corner
        p4 = corner[0, 3]  # Fourth corner
        
        angle = np.arctan2(p4[1] - p1[1], p1[0] - p4[0])
        return angle
    
    def _draw_marker_id(self, frame: np.ndarray, corner: np.ndarray, 
                       marker_id: int):
        """Draw marker ID on frame."""
        top_left = tuple(corner[0, 0].astype(int))
        cv2.putText(frame, str(marker_id), top_left, 
                   cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
    
    def draw_goals(self, frame: np.ndarray, goals: np.ndarray):
        """Draw goal positions on frame."""
        factor = int(14 * (self.height / self.width)) / self.marker_size
        
        for i in range(goals.shape[1]):
            goal_px = (goals[:2, i] * [1, -1] * factor * 3 + 
                      [self.width/2, self.height/2]).astype(int)
            cv2.circle(frame, tuple(goal_px), 7, (0, 200, 0), -1)
    
    def release(self):
        """Release camera resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("✓ Vision system released")
