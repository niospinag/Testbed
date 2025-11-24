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

@dataclass
class VisionConfig:
    """Vision system configuration."""
    camera_id: int = 0
    resolution: tuple = (1024, 576)
    marker_size: float = 10.2  # cm
    camera_config_path: str = "config/camera" # Ajusté la ruta por defecto
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
