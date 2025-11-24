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
