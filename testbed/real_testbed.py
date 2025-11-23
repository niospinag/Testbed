# ==========================================
# FILE: testbed/real_testbed.py
# ==========================================
"""
Real hardware testbed implementation with vision tracking and serial communication.
"""
import numpy as np
import cv2
import time
from typing import Optional

from core.base_testbed import BaseTestbed
from core.robot import Robot, RobotParameters
from hardware.vision import VisionSystem
from hardware.communication import SerialCommunicator, MultiSerialCommunicator
from config.settings import TestbedConfig
import utilities.controllers as ctrl
import utilities.misc as misc


class RealTestbed(BaseTestbed):
    """
    Real hardware testbed with camera tracking and serial communication.
    """
    
    def __init__(self, number_of_robots: int, 
                 show_figure: bool = True,
                 initial_conditions: Optional[np.ndarray] = None,
                 config: Optional[TestbedConfig] = None,
                 move_to_initial: bool = True):
        """
        Initialize real testbed.
        
        Args:
            number_of_robots: Number of robots to control
            show_figure: Show camera feed
            initial_conditions: Initial poses (3xN array)
            config: Testbed configuration
            move_to_initial: Move robots to initial conditions on startup
        """
        # Load configuration
        self.config = config or TestbedConfig()
        
        # Initialize base class
        super().__init__(number_of_robots, show_figure, initial_conditions, 
                        self.config.to_dict())
        
        # Initialize vision system
        self.vision = VisionSystem(
            camera_id=self.config.vision.camera_id,
            resolution=self.config.vision.resolution,
            marker_size=self.config.vision.marker_size,
            camera_config_path=self.config.vision.camera_config_path
        )
        
        # Initialize communication
        self.communicator = self._init_communication()
        
        # Video recording
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.recording = False
        
        # Goals visualization
        self.goals: Optional[np.ndarray] = None
        self.show_goals = False
        
        # Create robot objects
        self.robots = [
            Robot(i, self.poses[:, i], RobotParameters(**self.config.robot.__dict__))
            for i in range(number_of_robots)
        ]
        
        # Initial pose alignment
        if initial_conditions is not None and move_to_initial:
            print("Moving robots to initial conditions...")
            self._move_to_initial_conditions(initial_conditions)
        else:
            # Detect current positions
            self.poses = self._detect_initial_poses()
    
    def _default_config(self) -> dict:
        """Return default configuration."""
        return TestbedConfig().to_dict()
    
    def _generate_initial_poses(self) -> np.ndarray:
        """Generate initial poses by detecting robots."""
        return self._detect_initial_poses()
    
    def _init_communication(self):
        """Initialize serial communication based on number of robots."""
        comm_config = self.config.communication
        
        if self.number_of_robots <= comm_config.robots_per_antenna:
            # Single antenna
            return SerialCommunicator(
                port=comm_config.primary_port,
                baudrate=comm_config.baudrate
            )
        else:
            # Multiple antennas
            ports = [comm_config.primary_port, comm_config.secondary_port]
            return MultiSerialCommunicator(ports, comm_config.baudrate)
    
    def _detect_initial_poses(self) -> np.ndarray:
        """Detect initial robot poses from camera."""
        print("Detecting robots...")
        
        max_attempts = 10
        for attempt in range(max_attempts):
            poses_dict, frame = self.vision.detect_robots()
            
            if len(poses_dict) >= self.number_of_robots:
                # Convert dict to array
                poses = np.zeros((3, self.number_of_robots))
                for i in range(self.number_of_robots):
                    if (i + 1) in poses_dict:
                        poses[:, i] = poses_dict[i + 1]
                
                print(f"✓ Detected {len(poses_dict)} robots")
                return poses
            
            print(f"Attempt {attempt+1}/{max_attempts}: Only {len(poses_dict)} robots detected")
            time.sleep(0.5)
        
        raise RuntimeError(
            f"Could not detect {self.number_of_robots} robots. "
            f"Only found {len(poses_dict)}"
        )
    
    def _move_to_initial_conditions(self, target_poses: np.ndarray):
        """
        Move robots to initial conditions.
        
        Args:
            target_poses: Target 3xN poses
        """
        # Create controller for initial alignment
        controller = ctrl.create_reactive_pose_controller(
            linear_gain=[9, 0.1, 0],
            angular_gain=[14, 0.1, 1],
            num_robots=self.number_of_robots
        )
        
        max_iterations = 400
        position_tol = 10.0
        rotation_tol = 0.2
        
        for iteration in range(max_iterations):
            # Get current poses
            x = self.get_poses()
            
            # Check if all robots reached target
            at_target = misc.at_pose(x, target_poses, position_tol, rotation_tol)
            if np.size(at_target) == self.number_of_robots:
                print(f"✓ All robots at initial conditions (iter: {iteration})")
                break
            
            # Compute control
            dxu = controller(x, target_poses)
            self.set_velocities(np.arange(self.number_of_robots), dxu)
            
            # Execute
            self.step()
            
            if iteration % 50 == 0:
                print(f"Aligning robots... {np.size(at_target)}/{self.number_of_robots}")
        
        # Stop showing goals
        self.show_goals = False
        
        # Wait for user confirmation
        input('\n✓ Initial alignment complete. Press ENTER to start...\n')
        time.sleep(2)
    
    def get_poses(self) -> np.ndarray:
        """
        Get current robot poses from vision system.
        
        Returns:
            3xN array of [x, y, theta] poses
        """
        assert not self._checked_poses_already, \
            "Can only call get_poses() once per step()"
        
        # Update flags
        self._called_step_already = False
        self._checked_poses_already = True
        
        # Detect robots
        poses_dict, frame = self.vision.detect_robots()
        
        # Update poses
        for robot_id, pose in poses_dict.items():
            if 1 <= robot_id <= self.number_of_robots:
                self.poses[:, robot_id - 1] = pose
                self.robots[robot_id - 1].update_pose(pose)
        
        # Visualize
        if self.show_figure:
            self._visualize(frame)
        
        # Check for quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.call_at_scripts_end()
        
        return self.poses
    
    def _visualize(self, frame: np.ndarray):
        """Draw visualization overlay on frame."""
        # Draw coordinate axes
        self._draw_axes(frame)
        
        # Draw goals if enabled
        if self.show_goals and self.goals is not None:
            self.vision.draw_goals(frame, self.goals)
        
        # Display frame
        cv2.imshow("Testbed - Real", frame)
        
        # Record if enabled
        if self.recording and self.video_writer:
            self.video_writer.write(frame)
    
    def _draw_axes(self, frame: np.ndarray):
        """Draw coordinate axes on frame."""
        h, w = frame.shape[:2]
        color = (100, 100, 100)
        
        # Vertical axis
        cv2.line(frame, (w//2, 0), (w//2, h), color, 1)
        # Horizontal axis
        cv2.line(frame, (0, h//2), (w, h//2), color, 1)
    
    def step(self):
        """Execute one control step by sending velocities to robots."""
        assert not self._called_step_already, \
            "Must call get_poses() before step()"
        
        # Update flags
        self._called_step_already = True
        self._checked_poses_already = False
        
        # Validate states
        self._errors.update(self.validate_states())
        self._iterations += 1
        
        # Send velocities
        robot_ids = np.arange(self.number_of_robots)
        
        if isinstance(self.communicator, SerialCommunicator):
            self.communicator.send_velocities(robot_ids, self.velocities)
        else:
            self.communicator.send_velocities(robot_ids, self.velocities)
    
    def set_goals(self, goals: np.ndarray):
        """
        Set and visualize goal positions.
        
        Args:
            goals: 3xN or 2xN array of goal poses
        """
        assert isinstance(goals, np.ndarray), \
            f"goals must be np.ndarray, got {type(goals)}"
        
        self.goals = goals
        self.show_goals = True
        
        # Update robot objects
        for i in range(self.number_of_robots):
            self.robots[i].set_goal(goals[:, i] if goals.shape[0] == 3 else 
                                   np.append(goals[:, i], 0))
    
    def start_recording(self, filename: str):
        """
        Start recording video.
        
        Args:
            filename: Output filename (without extension)
        """
        import os
        
        # Create Videos directory if needed
        os.makedirs("Videos", exist_ok=True)
        
        fps = int(self.vision.cap.get(cv2.CAP_PROP_FPS))
        if fps == 0:
            fps = 30  # Default
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        output_path = f"Videos/{filename}.avi"
        
        self.video_writer = cv2.VideoWriter(
            output_path, fourcc, fps, 
            (self.vision.width, self.vision.height)
        )
        
        self.recording = True
        print(f"✓ Recording started: {output_path}")
    
    def stop_recording(self):
        """Stop recording video."""
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        self.recording = False
        print("✓ Recording stopped")
    
    def call_at_scripts_end(self):
        """Cleanup and print summary."""
        super().call_at_scripts_end()
        
        # Stop recording if active
        if self.recording:
            self.stop_recording()
        
        # Close communication
        self.communicator.close()
        
        # Release vision
        self.vision.release()
        
        exit()


# ==========================================
# Example usage
# ==========================================
if __name__ == "__main__":
    import utilities.controllers as ctrl
    
    # Configuration
    N = 3  # Number of robots
    
    # Initial conditions
    initial_conditions = np.array([
        [100, -100, 0],
        [50, -50, 0],
        [np.pi/2, -np.pi/2, 0]
    ])
    
    # Create testbed
    testbed = RealTestbed(
        number_of_robots=N,
        show_figure=True,
        initial_conditions=initial_conditions
    )
    
    # Start recording
    testbed.start_recording('experiment_001')
    
    # Define goals
    goals = initial_conditions * [[-1], [1], [-1]]
    testbed.set_goals(goals)
    
    # Create controller
    controller = ctrl.create_pid_unicycle_pose_controller(
        linear_gain=[10, 0.1, 0.2],
        angular_gain=[14, 0.1, 0.5],
        num_robots=N
    )
    
    # Control loop
    try:
        for iteration in range(500):
            # Get poses
            x = testbed.get_poses()
            
            # Check if done
            if all(robot.at_goal() for robot in testbed.robots):
                print(f"✓ All robots reached goals at iteration {iteration}")
                break
            
            # Compute control
            dxu = controller(x, goals)
            
            # Set velocities
            testbed.set_velocities(np.arange(N), dxu)
            
            # Execute
            testbed.step()
            
            # Status update
            if iteration % 50 == 0:
                at_goal = sum(1 for r in testbed.robots if r.at_goal())
                print(f"Iteration {iteration}: {at_goal}/{N} robots at goal")
    
    except KeyboardInterrupt:
        print("\n✓ Interrupted by user")
    
    finally:
        testbed.call_at_scripts_end()