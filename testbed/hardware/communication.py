import serial
import numpy as np
from typing import Optional

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
