# ==========================================
# FILE: core/base_testbed.py
# ==========================================
"""
Base abstract class for all testbed implementations.
Defines the common interface for virtual and real testbeds.
"""
from abc import ABC, abstractmethod
import numpy as np
import time


class BaseTestbed(ABC):
    """Abstract base class for testbed implementations."""
    
    def __init__(self, number_of_robots: int, show_figure: bool = True, 
                 initial_conditions: np.ndarray = None, config: dict = None):
        """
        Initialize the testbed.
        
        Args:
            number_of_robots: Number of robots to control
            show_figure: Whether to display visualization
            initial_conditions: Initial poses (3xN array)
            config: Configuration dictionary
        """
        self._validate_inputs(number_of_robots, initial_conditions)
        
        self.number_of_robots = number_of_robots
        self.show_figure = show_figure
        self.config = config or self._default_config()
        
        # Robot parameters
        self.time_step = self.config['robot']['time_step']
        self.robot_diameter = self.config['robot']['diameter']
        self.max_linear_velocity = self.config['robot']['max_linear_velocity']
        self.max_angular_velocity = self.config['robot']['max_angular_velocity']
        
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
        """Validate constructor inputs."""
        assert isinstance(number_of_robots, int), \
            f"number_of_robots must be int, got {type(number_of_robots)}"
        assert 0 < number_of_robots <= 30, \
            f"number_of_robots must be between 1-30, got {number_of_robots}"
        
        if initial_conditions is not None:
            assert isinstance(initial_conditions, np.ndarray), \
                f"initial_conditions must be np.ndarray, got {type(initial_conditions)}"
            assert initial_conditions.shape == (3, number_of_robots), \
                f"initial_conditions must be (3, {number_of_robots}), got {initial_conditions.shape}"
    
    @abstractmethod
    def _default_config(self) -> dict:
        """Return default configuration."""
        pass
    
    @abstractmethod
    def _generate_initial_poses(self) -> np.ndarray:
        """Generate initial robot poses."""
        pass
    
    @abstractmethod
    def get_poses(self) -> np.ndarray:
        """
        Get current robot poses.
        
        Returns:
            3xN array of [x, y, theta] poses
        """
        pass
    
    @abstractmethod
    def step(self):
        """Execute one simulation/control step."""
        pass
    
    def set_velocities(self, ids: np.ndarray, velocities: np.ndarray):
        """
        Set robot velocities with saturation.
        
        Args:
            ids: Robot indices
            velocities: 2xN array of [v, w] velocities
        """
        # Saturate linear velocities
        idxs = np.where(np.abs(velocities[0, :]) > self.max_linear_velocity)
        velocities[0, idxs] = self.max_linear_velocity * np.sign(velocities[0, idxs])
        
        # Saturate angular velocities
        idxs = np.where(np.abs(velocities[1, :]) > self.max_angular_velocity)
        velocities[1, idxs] = self.max_angular_velocity * np.sign(velocities[1, idxs])
        
        self.velocities = velocities
    
    def validate_states(self) -> dict:
        """
        Validate current robot states (boundaries, collisions).
        
        Returns:
            Dictionary of errors
        """
        errors = {}
        p = self.poses
        boundaries = self.config['arena']['boundaries']
        
        # Check boundaries
        for i in range(self.number_of_robots):
            x, y = p[0, i], p[1, i]
            if not (boundaries[0] <= x <= boundaries[1] and 
                   boundaries[2] <= y <= boundaries[3]):
                errors['boundary'] = errors.get('boundary', 0) + 1
        
        # Check collisions
        for i in range(self.number_of_robots - 1):
            for j in range(i + 1, self.number_of_robots):
                dist = np.linalg.norm(p[:2, i] - p[:2, j])
                if dist <= self.robot_diameter:
                    errors['collision'] = errors.get('collision', 0) + 1
        
        return errors
    
    def call_at_scripts_end(self):
        """Print summary statistics and cleanup."""
        print('\n' + '='*50)
        print('TESTBED SUMMARY')
        print('='*50)
        print(f'Total iterations: {self._iterations}')
        print(f'Total time: {self._iterations * self.time_step:.2f}s')
        
        if self._errors:
            print('\nErrors detected:')
            for error_type, count in self._errors.items():
                print(f'  - {error_type}: {count}')
        else:
            print('\n✓ No errors detected!')
        
        print('='*50 + '\n')


# ==========================================
# FILE: core/robot.py
# ==========================================
"""
Robot class representing individual robot state and parameters.
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class RobotParameters:
    """Physical robot parameters."""
    diameter: float = 20.0
    wheel_radius: float = 3.0
    base_length: float = 11.0
    max_linear_velocity: float = 300.0
    max_angular_velocity: float = 45.0


class Robot:
    """Represents a single robot."""
    
    def __init__(self, robot_id: int, initial_pose: np.ndarray = None, 
                 params: RobotParameters = None):
        """
        Initialize robot.
        
        Args:
            robot_id: Unique robot identifier
            initial_pose: Initial [x, y, theta] pose
            params: Robot physical parameters
        """
        self.id = robot_id
        self.params = params or RobotParameters()
        
        # State
        self.pose = initial_pose if initial_pose is not None else np.zeros(3)
        self.velocity = np.zeros(2)  # [v, w]
        
        # Tracking
        self.goal = None
        self.trajectory = []
    
    @property
    def position(self) -> np.ndarray:
        """Get [x, y] position."""
        return self.pose[:2]
    
    @property
    def orientation(self) -> float:
        """Get theta orientation."""
        return self.pose[2]
    
    def update_pose(self, new_pose: np.ndarray):
        """Update robot pose and store in trajectory."""
        self.pose = new_pose
        self.trajectory.append(new_pose.copy())
    
    def set_goal(self, goal: np.ndarray):
        """Set target goal."""
        self.goal = goal
    
    def at_goal(self, position_tol: float = 10.0, 
                angle_tol: float = 0.2) -> bool:
        """Check if robot reached its goal."""
        if self.goal is None:
            return False
        
        pos_error = np.linalg.norm(self.position - self.goal[:2])
        angle_error = abs(np.arctan2(
            np.sin(self.orientation - self.goal[2]),
            np.cos(self.orientation - self.goal[2])
        ))
        
        return pos_error < position_tol and angle_error < angle_tol


# ==========================================
# FILE: hardware/communication.py
# ==========================================
"""
Serial communication with robot hardware.
"""
import serial
import numpy as np
from typing import Optional


class SerialCommunicator:
    """Handles serial communication with ESP8266 modules."""
    
    def __init__(self, port: str, baudrate: int = 115200, 
                 timeout: float = 1.0):
        """
        Initialize serial connection.
        
        Args:
            port: Serial port (e.g., 'COM4' or '/dev/ttyUSB0')
            baudrate: Communication speed
            timeout: Read timeout
        """
        self.port = port
        self.baudrate = baudrate
        self.connection = serial.Serial(port, baudrate, timeout=timeout)
        print(f"✓ Serial connection established on {port}")
    
    def send_velocities(self, robot_ids: np.ndarray, velocities: np.ndarray,
                       led_colors: tuple = (0, 0, 10)):
        """
        Send velocity commands to robots.
        
        Args:
            robot_ids: Array of robot IDs
            velocities: 2xN array of [v, w] velocities
            led_colors: RGB tuple for LED colors
        """
        num_robots = len(robot_ids)
        
        # Build command string
        command = f"{num_robots}\n"
        
        for i, robot_id in enumerate(robot_ids):
            v, w = velocities[0, i], velocities[1, i]
            r, g, b = led_colors
            command += f"{robot_id+1}; {v:.0f}; {w:.0f}; {r}; {g}; {b}\n"
        
        command += "xxF"
        
        # Send
        self.connection.write(command.encode("utf-8"))
    
    def close(self):
        """Close serial connection."""
        if self.connection and self.connection.is_open:
            self.connection.close()
            print(f"✓ Serial connection closed on {self.port}")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class MultiSerialCommunicator:
    """Manages multiple serial connections for >6 robots."""
    
    def __init__(self, ports: list, baudrate: int = 115200):
        """
        Initialize multiple serial connections.
        
        Args:
            ports: List of serial ports
            baudrate: Communication speed
        """
        self.communicators = [
            SerialCommunicator(port, baudrate) for port in ports
        ]
        self.robots_per_comm = 6  # Max robots per antenna
    
    def send_velocities(self, robot_ids: np.ndarray, velocities: np.ndarray):
        """Send velocities to robots across multiple communicators."""
        num_robots = len(robot_ids)
        
        for i, comm in enumerate(self.communicators):
            start_idx = i * self.robots_per_comm
            end_idx = min(start_idx + self.robots_per_comm, num_robots)
            
            if start_idx >= num_robots:
                break
            
            ids = robot_ids[start_idx:end_idx]
            vels = velocities[:, start_idx:end_idx]
            comm.send_velocities(ids, vels)
    
    def close(self):
        """Close all serial connections."""
        for comm in self.communicators:
            comm.close()


# ==========================================
# FILE: hardware/vision.py
# ==========================================
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


# ==========================================
# FILE: config/settings.py
# ==========================================
"""
Configuration management for testbed.
"""
from dataclasses import dataclass, field
from typing import List
import platform


@dataclass
class RobotConfig:
    """Robot physical parameters."""
    time_step: float = 0.033
    diameter: float = 20.0
    wheel_radius: float = 3.0
    base_length: float = 11.0
    max_linear_velocity: float = 300.0
    max_angular_velocity: float = 45.0


@dataclass
class ArenaConfig:
    """Arena/workspace configuration."""
    boundaries: List[float] = field(default_factory=lambda: [-200, 200, -150, 150])
    # boundaries = [x_min, x_max, y_min, y_max]


@dataclass
class VisionConfig:
    """Vision system configuration."""
    camera_id: int = 0
    resolution: tuple = (1024, 576)
    marker_size: float = 10.2  # cm
    camera_config_path: str = "Camera"
    exposure: int = 0


@dataclass
class CommunicationConfig:
    """Serial communication configuration."""
    primary_port: str = field(
        default_factory=lambda: "COM4" if platform.system() == "Windows" 
        else "/dev/ttyUSB0"
    )
    secondary_port: str = field(
        default_factory=lambda: "COM5" if platform.system() == "Windows" 
        else "/dev/ttyUSB1"
    )
    baudrate: int = 115200
    robots_per_antenna: int = 6


@dataclass
class TestbedConfig:
    """Complete testbed configuration."""
    robot: RobotConfig = field(default_factory=RobotConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'robot': self.robot.__dict__,
            'arena': self.arena.__dict__,
            'vision': self.vision.__dict__,
            'communication': self.communication.__dict__
        }


# Example usage:
if __name__ == "__main__":
    config = TestbedConfig()
    print("Default configuration:")
    print(config.to_dict())