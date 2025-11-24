# Multi-Robot Testbed Platform

[![Python Version](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey.svg)](https://github.com/yourusername/testbed)

> [🇪🇸 Versión en Español](#versión-en-español) | [🇬🇧 English Version](#english-version)

---

## 🇬🇧 English Version

A comprehensive Python framework for simulating and deploying multi-robot control systems with real-time vision tracking using ArUco markers.

### 🎯 Features

- **🤖 Dual Operation Modes**: Seamless switching between Virtual Simulation and Real Hardware.
- **👁️ Vision-Based Tracking**: Real-time pose estimation using ArUco markers and OpenCV.
- **🎮 Modular Architecture**: Clean separation between core logic, hardware interfaces, and control algorithms.
- **🛡️ Safety Mechanisms**: Barrier certificates for collision avoidance.
- **📊 Multi-Robot Support**: Control up to 30 robots simultaneously.
- **📈 Trajectory Tracking**: Load and follow predefined paths from MATLAB (`.mat`) data.
- **🎥 Video Recording**: Automatic recording of experiments (ignores new files in Git).

### 📋 Table of Contents

- [Installation](#installation)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Hardware Setup](#hardware-setup)
- [Troubleshooting](#troubleshooting)
- [Citation](#citation)

---

## 🚀 Installation

### Prerequisites

- **Software**: Python 3.10+, pip, Git.
- **Hardware** (optional): USB Camera, ESP8266 modules, Mobile robots (differential drive).

### Setup Steps

1. **Clone the repository**:
```bash
git clone https://github.com/niospinag/testbed.git
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

4. **Install the package in "Editable" mode (Recommended)**:
   *This allows you to edit the code in `testbed/` and see changes immediately without reinstalling.*
```bash
pip install -e .
```

---

## 📂 Project Structure

The project follows a modular package architecture:

```text
Testbed/
├── assets/                 # Markers and static resources
├── config/                 # Camera calibration files
├── data/                   # Trajectories (.mat) and Results (.csv)
├── examples/               # Ready-to-run scripts (Start here!)
│   ├── basic_simulation.py # Main entry point example
│   └── ...
├── testbed/                # MAIN PACKAGE
│   ├── config/             # Configuration classes (settings.py)
│   ├── control/            # Controllers (PID, CLF) & Barriers
│   ├── core/               # Base classes and Robot definitions
│   ├── hardware/           # Real robot interface & Vision system
│   ├── simulators/         # Virtual simulation & Plotting logic
│   └── utils/              # I/O, Geometry, and Transformations
├── videos/                 # Output folder for recordings
├── requirements.txt
└── setup.py
```

---

## 🎮 Quick Start

To run your first simulation, navigate to the `examples` folder.

**File:** `examples/basic_simulation.py`

```python
import sys
from pathlib import Path
import numpy as np

# Add project root to path (if not installed via pip -e .)
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# --- NEW IMPORTS ---
from testbed import VirtualTestbed
from testbed.utils import io, geometry
import testbed.control.controllers as ctrl

# 1. Load Data
file_path = 'data/trajectories/data_7v_7N.mat'
load_pos = io.load_data_matlab(str(project_root / file_path), split_data=10)
initial_conditions = load_pos(0)[:, :3]

# 2. Initialize Virtual Environment
env = VirtualTestbed(number_of_robots=3, show_figure=True, initial_conditions=initial_conditions)

# 3. Create Controller
controller = ctrl.create_pid_unicycle_pose_controller(num_robots=3)

# 4. Loop
x = env.get_poses()
env.step()

while True:
    x = env.get_poses()
    goals = load_pos(0)[:, :3]
    
    # Compute control
    dxu = controller(x, goals)
    
    # Apply velocities
    env.set_velocities(np.arange(3), dxu)
    env.step()
```

**Run it via terminal:**
```bash
python3 examples/basic_simulation.py
```

---

## ⚙️ Configuration

Global settings are managed in **`testbed/config/settings.py`**. You can modify:

*   **Robot Parameters**: Size, wheel radius, max velocity.
*   **Arena**: Boundaries (`[x_min, x_max, y_min, y_max]`).
*   **Vision**: Camera ID, Resolution, Marker size.
*   **Communication**: Serial ports (COM4/ttyUSB0) and baudrate.

---

## 🔧 Hardware Setup

If you are deploying to real robots using `RealTestbed`:

1.  **Camera**: Ensure your camera calibration files (`cameraMatrix.txt`, `cameraDistortion.txt`) are in `config/camera/`.
2.  **Serial Port**: Check your USB connection.
    *   *Linux*: `/dev/ttyUSB0` (Remember to grant permissions: `sudo chmod 666 /dev/ttyUSB0`).
    *   *Windows*: `COM3` or `COM4`.
3.  **Markers**: Use the ArUco dictionary `DICT_4X4_100`.

---

## 🐛 Troubleshooting

*   **`ModuleNotFoundError: No module named 'testbed'`**:
    *   Ensure you ran `pip install -e .` in the root directory.
    *   Or ensure your script adds the parent directory to `sys.path`.

*   **`FileNotFoundError: ... data_7v_7N.mat`**:
    *   Ensure you are running the script from the correct directory or using absolute paths (as shown in `basic_simulation.py`).

---

## 📝 Citation

If you use this platform, please cite:

```bibtex
@software{testbed2024,
  title={Multi-Robot Testbed Platform},
  author={Nestor Ivan Ospina},
  year={2024},
  url={https://github.com/niospinag/testbed}
}
```

---
---
---

# 🇪🇸 Versión en Español

## Plataforma Testbed Multi-Robot

Un framework completo en Python para simular y desplegar sistemas de control multi-robot, con seguimiento visual en tiempo real mediante marcadores ArUco.

### 🎯 Características

- **🤖 Modos Duales**: Cambio transparente entre Simulación Virtual y Hardware Real.
- **👁️ Visión Artificial**: Estimación de pose en tiempo real usando ArUco y OpenCV.
- **🎮 Arquitectura Modular**: Separación limpia entre lógica central (`core`), hardware (`hardware`), utilidades (`utils`) y control.
- **🛡️ Seguridad**: Certificados de barrera (Barrier Certificates) para evitar colisiones.
- **📈 Seguimiento de Trayectorias**: Carga datos de MATLAB (`.mat`) y sigue rutas complejas.
- **🎥 Grabación**: Sistema automático de grabación de experimentos.

### 📋 Tabla de Contenidos

- [Instalación](#instalación)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Inicio Rápido](#inicio-rápido)
- [Configuración](#configuración)
- [Hardware](#configuración-de-hardware)

---

## 🚀 Instalación

### Requisitos

- Python 3.10+, pip, Git.

### Pasos

1. **Clonar repositorio**:
```bash
git clone https://github.com/niospinag/testbed.git
cd testbed
```

2. **Entorno Virtual**:
```bash
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

3. **Instalar dependencias**:
```bash
pip install -r requirements.txt
```

4. **Instalar el paquete en modo "Editable" (Recomendado)**:
   *Esto permite importar `testbed` desde cualquier lugar sin romper las rutas.*
```bash
pip install -e .
```

---

## 📂 Estructura del Proyecto

El proyecto ha sido refactorizado para ser modular:

```text
Testbed/
├── assets/                 # Marcadores y recursos
├── config/                 # Calibración de cámara
├── data/                   # Trayectorias (.mat) y Resultados
├── examples/               # Scripts ejecutables (¡Empieza aquí!)
│   ├── basic_simulation.py # Ejemplo principal
│   └── ...
├── testbed/                # PAQUETE PRINCIPAL
│   ├── config/             # Configuración global (settings.py)
│   ├── control/            # Controladores y Barreras
│   ├── core/               # Clases base y Robot
│   ├── hardware/           # Interfaz Real y Visión
│   ├── simulators/         # Simulador Virtual y Gráficos
│   └── utils/              # I/O, Geometría y Transformaciones
├── videos/                 # Salida de videos
└── setup.py
```

---

## 🎮 Inicio Rápido

Para correr una simulación, usa el script en la carpeta `examples`.

**Ejecutar:**
```bash
python3 examples/basic_simulation.py
```

**Ejemplo de Código (Resumido):**

```python
from testbed import VirtualTestbed
from testbed.utils import io, geometry
import testbed.control.controllers as ctrl

# 1. Cargar Datos
load_pos = io.load_data_matlab('data/trajectories/data_7v_7N.mat', split_data=10)
initial_conditions = load_pos(0)[:, :3]

# 2. Iniciar Simulador
env = VirtualTestbed(number_of_robots=3, show_figure=True, initial_conditions=initial_conditions)

# 3. Controlador
controller = ctrl.create_pid_unicycle_pose_controller(num_robots=3)

# 4. Bucle
while True:
    x = env.get_poses()
    dxu = controller(x, metas)
    env.set_velocities(range(3), dxu)
    env.step()
```

---

## ⚙️ Configuración

Toda la configuración del sistema se centraliza en **`testbed/config/settings.py`**.
Aquí puedes ajustar:

*   **Física del Robot**: Radio de rueda, velocidad máxima.
*   **Arena**: Límites del espacio de trabajo.
*   **Visión**: ID de cámara (0, 1, 2), resolución.
*   **Comunicación**: Puertos seriales (`COM4`, `/dev/ttyUSB0`).

---

## 🔧 Configuración de Hardware

Si vas a usar el **`RealTestbed`**:

1.  **Cámara**: Asegúrate de tener los archivos `cameraMatrix.txt` y `cameraDistortion.txt` en `config/camera/`.
2.  **Permisos (Linux)**:
    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```
3.  **Marcadores**: Imprime los marcadores de `assets/markers/`.

---

## 📧 Contacto

- **Autor**: Nestor Ivan Ospina Gaitan
- **Email**: niospinag@unal.edu.co
- **Issues**: Por favor reportar bugs en la pestaña de Issues.

---

<p align="center">
  Made with ❤️ for robotics research
</p>