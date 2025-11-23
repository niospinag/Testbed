# Multi-Robot Testbed Platform

[![Python Version](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey.svg)](https://github.com/yourusername/testbed)

> [🇪🇸 Versión en Español](#versión-en-español) | [🇬🇧 English Version](#english-version)

---

## 🇬🇧 English Version

A comprehensive Python framework for simulating and deploying multi-robot control systems with real-time vision tracking using ArUco markers.

### 🎯 Features

- **🤖 Dual Operation Modes**: Virtual simulation and real hardware implementation
- **👁️ Vision-Based Tracking**: Real-time pose estimation using ArUco markers and OpenCV
- **🎮 Multiple Controllers**: PID, reactive, and CLF-based control strategies
- **🛡️ Safety Mechanisms**: Barrier certificates for collision avoidance
- **📊 Multi-Robot Support**: Control up to 30 robots simultaneously
- **📈 Trajectory Tracking**: Load and follow predefined paths from MATLAB data
- **🎥 Video Recording**: Built-in capability to record experiments

### 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Usage Examples](#usage-examples)
- [Project Structure](#project-structure)
- [Controllers](#available-controllers)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Citation](#citation)

---

## 🚀 Installation

### Prerequisites

- **Software**:
  - Python 3.10 or higher
  - pip package manager
  - Git

- **Hardware** (for real experiments):
  - USB camera (webcam or external)
  - ESP8266 WiFi modules (for robot communication)
  - ArUco markers (4x4_100 dictionary, 10.2cm size)
  - Mobile robots with differential drive

### Setup Steps

1. **Clone the repository**:
```bash
git clone https://github.com/yourusername/testbed.git
cd testbed
```

2. **Create a virtual environment**:
```bash
# Linux/Mac
python3 -m venv venv
source venv/bin/activate

# Windows
python -m venv venv
venv\Scripts\activate
```

3. **Install dependencies**:
```bash
pip install -r requirements.txt
```

4. **Install as a package** (optional):
```bash
pip install -e .
```

---

## 🎮 Quick Start

### Virtual Simulation

```python
from testbed_virt import Testbed
import utilities.controllers as ctrl
import utilities.misc as misc
import numpy as np

# Initialize testbed with 3 robots
N = 3
initial_conditions = misc.generate_initial_conditions(N)
testbed = Testbed(
    number_of_robots=N,
    show_figure=True,
    initial_conditions=initial_conditions
)

# Create PID controller
controller = ctrl.create_pid_unicycle_pose_controller(
    linear_gain=[10, 0.1, 0.2],
    angular_gain=[14, 0.1, 0.5],
    num_robots=N
)

# Define goal positions
goals = np.array([[100, -100, 0], 
                  [50, -50, 0], 
                  [np.pi/2, -np.pi/2, 0]])

# Control loop
for _ in range(500):
    x = testbed.get_poses()
    dxu = controller(x, goals)
    testbed.set_velocities(np.arange(N), dxu)
    testbed.step()

testbed.call_at_scripts_end()
```

### Real Hardware

```python
from testbed_real import Testbed

# Same code as above, but uses real robots!
testbed = Testbed(
    number_of_robots=3,
    show_figure=True,
    initial_conditions=initial_conditions
)

# Enable video recording
testbed.record_video('my_experiment')

# ... control loop
```

---

## 🔧 Hardware Setup

### Camera Configuration

The camera ID can be configured in `testbed_real.py` (line 81):

```python
self.cap = cv2.VideoCapture(0)  # Camera selection
```

**Camera IDs**:
- `0`: Built-in/front camera
- `1`: External/back camera  
- `2`: Secondary external camera

### Serial Communication

Configure serial ports for ESP8266 communication:

#### Windows
```python
# In testbed_real.py (lines 105-108)
self.esp8266 = serial.Serial("COM4", 115200)
if number_of_robots > 6:
    self.esp8266_2 = serial.Serial("COM5", 115200)
```

#### Linux/Ubuntu
```python
self.esp8266 = serial.Serial("/dev/ttyUSB0", 115200)
if number_of_robots > 6:
    self.esp8266_2 = serial.Serial("/dev/ttyUSB1", 115200)
```

**Note**: For more than 6 robots, you need to connect a second antenna.

#### Grant Serial Port Permissions (Linux)
```bash
sudo chmod 666 /dev/ttyUSB0
# Or permanently:
sudo usermod -a -G dialout $USER
# Then logout and login
```

---

## ⚙️ Configuration

### Camera Calibration

1. Place calibration files in the `Camera/` directory:
   - `cameraMatrix.txt`: Camera intrinsic matrix
   - `cameraDistortion.txt`: Distortion coefficients

2. Generate calibration using OpenCV calibration tools or provided scripts.

### Robot Parameters

Edit in `testbed_real.py` or `testbed_virt.py`:

```python
self.robot_diameter = 20      # cm
self.wheel_radius = 3         # cm
self.base_length = 11         # cm
self.max_linear_velocity = 300   # units/s
self.max_angular_velocity = 45   # rad/s
```

### Arena Boundaries

```python
self.boundaries = [-200, -150, 200, 150]  # [x_min, y_min, x_max, y_max]
```

---

## 📚 Usage Examples

### 1. Load Trajectory Data

Modify `utilities/misc.py` to load your data format:

```python
# Load MATLAB data
load_position = misc.load_data_matlab(
    'data/my_trajectory.mat',
    split_data=10,
    shift_x=0,
    scale_x=1
)

# Use in control loop
for i in range(num_steps):
    goal_points = load_position(i)[:, :N]
    # ... control logic
```

### 2. Create Custom Controller

Add to `utilities/controllers.py`:

```python
def create_my_controller(gain=1.0, num_robots=1):
    """My custom controller."""
    def controller(states, goals):
        # Your control law
        velocities = compute_control(states, goals, gain)
        return velocities
    return controller
```

### 3. Enable Safety Barriers

```python
from utilities.barrier_certificates import (
    create_unicycle_barrier_certificate_with_boundary
)

# Create barrier certificate
barrier = create_unicycle_barrier_certificate_with_boundary(
    barrier_gain=100,
    safety_radius=17,
    boundary_points=np.array([-200, 200, -150, 150])
)

# Apply in control loop
dxu = controller(x, goals)
dxu_safe = barrier(dxu, x)  # Safe velocities
testbed.set_velocities(np.arange(N), dxu_safe)
```

### 4. Record Video

```python
# Start recording
testbed.record_video('experiment_name')

# Control loop...

# Video automatically saved in Videos/ folder
testbed.call_at_scripts_end()
```

---

## 📁 Project Structure

```
Testbed/
├── testbed_real.py         # Real hardware implementation
├── testbed_virt.py         # Virtual simulation
├── testbed.py              # Base class (ABC)
├── plotlab.py              # Visualization tools
│
├── utilities/              # Control & vision utilities
│   ├── controllers.py      # Control algorithms
│   ├── barrier_certificates.py  # Safety mechanisms
│   ├── misc.py            # Helper functions
│   ├── transformations.py # Coordinate transforms
│   └── ArucoModule.py     # Computer vision
│
├── Camera/                 # Camera calibration
│   ├── cameraMatrix.txt
│   └── cameraDistortion.txt
│
├── Markers/                # ArUco marker images
├── data/                   # Experimental data (.mat, .csv)
├── Videos/                 # Recorded videos
│
├── examples/               # Example scripts
│   ├── thesis_example.py
│   ├── control_example.py
│   ├── tune_pid.py
│   └── tune_pid_regression.py
│
├── requirements.txt        # Python dependencies
└── README.md              # This file
```

---

## 🎛️ Available Controllers

### 1. PID Position Controller
```python
controller = ctrl.create_pid_unicycle_position_controller(
    linear_gain=[kp, ki, kd],
    angular_gain=[kp, ki, kd],
    num_robots=N
)
```

### 2. PID Pose Controller
```python
controller = ctrl.create_pid_unicycle_pose_controller(
    linear_gain=[10, 0.1, 0.2],
    angular_gain=[14, 0.1, 0.5],
    num_robots=N
)
```

### 3. Reactive Pose Controller (Collision Avoidance)
```python
controller = ctrl.create_reactive_pose_controller(
    linear_gain=[9, 0.1, 0],
    angular_gain=[14, 0.1, 1],
    num_robots=N
)
```

### 4. CLF-Based Controllers
- `create_clf_unicycle_position_controller`
- `create_clf_unicycle_pose_controller`
- `create_hybrid_unicycle_pose_controller`

---

## 🐛 Troubleshooting

### Camera Not Detected
```bash
# List available cameras
ls /dev/video*

# Test camera
python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### Serial Port Access Denied (Linux)
```bash
# Check permissions
ls -l /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login

# Or temporary fix
sudo chmod 666 /dev/ttyUSB0
```

### ArUco Markers Not Detected
- Ensure proper lighting (avoid shadows)
- Check marker size matches configuration (10.2cm)
- Verify camera calibration files
- Clean marker surfaces

### Import Errors
```bash
# Reinstall dependencies
pip install --force-reinstall -r requirements.txt

# Check Python version
python --version  # Should be 3.10+
```

---

## 🧪 Testing

Run example scripts to verify installation:

```bash
# Virtual simulation
python thesis_example.py

# PID tuning (generates heatmap)
python tune_pid.py

# Control example
python control_example.py
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style
- Follow PEP 8 guidelines
- Add docstrings to functions
- Include type hints where possible
- Write descriptive commit messages

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📝 Citation

If you use this testbed in your research, please cite:

```bibtex
@software{testbed2024,
  title={Multi-Robot Testbed Platform},
  author={Your Name},
  year={2024},
  url={https://github.com/yourusername/testbed},
  version={1.0}
}
```

---

## 🙏 Acknowledgments

- Based on the [Robotarium](https://www.robotarium.gatech.edu/) framework
- Uses [OpenCV](https://opencv.org/) for computer vision
- ArUco marker detection via opencv-contrib
- Barrier certificates adapted from [Georgia Tech GRITS Lab](https://www.davidanisi.com/)

---

## 📧 Contact & Support

- **Author**: Your Name
- **Email**: your.email@example.com
- **Issues**: [GitHub Issues](https://github.com/yourusername/testbed/issues)
- **Discussions**: [GitHub Discussions](https://github.com/yourusername/testbed/discussions)

---

## 🛣️ Roadmap

- [ ] ROS2 integration
- [ ] Web-based visualization dashboard
- [ ] Automatic camera calibration tool
- [ ] Docker containerization
- [ ] Real-time telemetry dashboard
- [ ] Multi-camera support
- [ ] Gazebo simulation integration

---

## 📊 System Requirements

### Minimum
- CPU: Dual-core 2.0 GHz
- RAM: 4 GB
- Camera: 640x480 @ 30fps
- OS: Ubuntu 20.04+, Windows 10+

### Recommended  
- CPU: Quad-core 2.5 GHz+
- RAM: 8 GB+
- Camera: 1920x1080 @ 60fps
- GPU: Optional (for advanced visualization)
- OS: Ubuntu 22.04

---

<p align="center">
  Made with ❤️ for robotics research
</p>

---
---
---

# 🇪🇸 Versión en Español

## Plataforma Testbed Multi-Robot

Framework completo en Python para simular y desplegar sistemas de control multi-robot con seguimiento visual en tiempo real usando marcadores ArUco.

### 🎯 Características

- **🤖 Modos de Operación Duales**: Simulación virtual e implementación en hardware real
- **👁️ Seguimiento Basado en Visión**: Estimación de pose en tiempo real usando marcadores ArUco y OpenCV
- **🎮 Múltiples Controladores**: Estrategias de control PID, reactivo y basado en CLF
- **🛡️ Mecanismos de Seguridad**: Certificados de barrera para evitar colisiones
- **📊 Soporte Multi-Robot**: Controla hasta 30 robots simultáneamente
- **📈 Seguimiento de Trayectorias**: Carga y sigue trayectorias predefinidas desde datos MATLAB
- **🎥 Grabación de Video**: Capacidad integrada para grabar experimentos

---

## 🚀 Instalación

### Requisitos Previos

- **Software**:
  - Python 3.10 o superior
  - Gestor de paquetes pip
  - Git

- **Hardware** (para experimentos reales):
  - Cámara USB (webcam o externa)
  - Módulos WiFi ESP8266 (para comunicación con robots)
  - Marcadores ArUco (diccionario 4x4_100, tamaño 10.2cm)
  - Robots móviles con tracción diferencial

### Pasos de Configuración

1. **Clonar el repositorio**:
```bash
git clone https://github.com/tuusuario/testbed.git
cd testbed
```

2. **Crear entorno virtual**:
```bash
# Linux/Mac
python3 -m venv venv
source venv/bin/activate

# Windows
python -m venv venv
venv\Scripts\activate
```

3. **Instalar dependencias**:
```bash
pip install -r requirements.txt
```

4. **Instalar como paquete** (opcional):
```bash
pip install -e .
```

---

## 🎮 Inicio Rápido

### Simulación Virtual

```python
from testbed_virt import Testbed
import utilities.controllers as ctrl
import utilities.misc as misc
import numpy as np

# Inicializar testbed con 3 robots
N = 3
condiciones_iniciales = misc.generate_initial_conditions(N)
testbed = Testbed(
    number_of_robots=N,
    show_figure=True,
    initial_conditions=condiciones_iniciales
)

# Crear controlador PID
controlador = ctrl.create_pid_unicycle_pose_controller(
    linear_gain=[10, 0.1, 0.2],
    angular_gain=[14, 0.1, 0.5],
    num_robots=N
)

# Definir posiciones objetivo
metas = np.array([[100, -100, 0], 
                  [50, -50, 0], 
                  [np.pi/2, -np.pi/2, 0]])

# Lazo de control
for _ in range(500):
    x = testbed.get_poses()
    dxu = controlador(x, metas)
    testbed.set_velocities(np.arange(N), dxu)
    testbed.step()

testbed.call_at_scripts_end()
```

---

## 🔧 Configuración de Hardware

### Configuración de Cámara

El ID de cámara se configura en `testbed_real.py` (línea 81):

```python
self.cap = cv2.VideoCapture(0)  # Selección de cámara
```

**IDs de Cámara**:
- `0`: Cámara integrada/frontal
- `1`: Cámara externa/trasera
- `2`: Cámara externa secundaria

### Comunicación Serial

Configurar puertos seriales para comunicación ESP8266:

#### Windows
```python
# En testbed_real.py (líneas 105-108)
self.esp8266 = serial.Serial("COM4", 115200)
if number_of_robots > 6:
    self.esp8266_2 = serial.Serial("COM5", 115200)
```

#### Linux/Ubuntu
```python
self.esp8266 = serial.Serial("/dev/ttyUSB0", 115200)
if number_of_robots > 6:
    self.esp8266_2 = serial.Serial("/dev/ttyUSB1", 115200)
```

**Nota**: Para más de 6 robots, necesitas conectar una segunda antena.

#### Otorgar Permisos de Puerto Serial (Linux)
```bash
sudo chmod 666 /dev/ttyUSB0
# O permanentemente:
sudo usermod -a -G dialout $USER
# Luego cerrar sesión e iniciar nuevamente
```

---

## ⚙️ Configuración

### Calibración de Cámara

1. Coloca archivos de calibración en el directorio `Camera/`:
   - `cameraMatrix.txt`: Matriz intrínseca de cámara
   - `cameraDistortion.txt`: Coeficientes de distorsión

2. Genera calibración usando herramientas de OpenCV o scripts provistos.

### Parámetros del Robot

Editar en `testbed_real.py` o `testbed_virt.py`:

```python
self.robot_diameter = 20      # cm
self.wheel_radius = 3         # cm  
self.base_length = 11         # cm
self.max_linear_velocity = 300   # unidades/s
self.max_angular_velocity = 45   # rad/s
```

### Límites del Arena

```python
self.boundaries = [-200, -150, 200, 150]  # [x_min, y_min, x_max, y_max]
```

---

## 📚 Ejemplos de Uso

### 1. Cargar Datos de Trayectoria

Modificar `utilities/misc.py` para cargar tu formato de datos:

```python
# Cargar datos MATLAB
cargar_posicion = misc.load_data_matlab(
    'data/mi_trayectoria.mat',
    split_data=10,
    shift_x=0,
    scale_x=1
)

# Usar en lazo de control
for i in range(num_pasos):
    puntos_meta = cargar_posicion(i)[:, :N]
    # ... lógica de control
```

### 2. Grabar Video

```python
# Iniciar grabación
testbed.record_video('nombre_experimento')

# Lazo de control...

# Video guardado automáticamente en carpeta Videos/
testbed.call_at_scripts_end()
```

---

## 🐛 Solución de Problemas

### Cámara No Detectada
```bash
# Listar cámaras disponibles
ls /dev/video*

# Probar cámara
python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### Acceso Denegado al Puerto Serial (Linux)
```bash
# Verificar permisos
ls -l /dev/ttyUSB0

# Agregar usuario al grupo dialout
sudo usermod -a -G dialout $USER
# Cerrar sesión e iniciar nuevamente

# O solución temporal
sudo chmod 666 /dev/ttyUSB0
```

### Marcadores ArUco No Detectados
- Asegurar iluminación adecuada (evitar sombras)
- Verificar que tamaño de marcador coincide con configuración (10.2cm)
- Verificar archivos de calibración de cámara
- Limpiar superficies de marcadores

---

## 📧 Contacto y Soporte

- **Autor**: Tu Nombre
- **Email**: tu.email@ejemplo.com
- **Problemas**: [GitHub Issues](https://github.com/tuusuario/testbed/issues)

---

<p align="center">
  Hecho con ❤️ para investigación en robótica
</p>