#!/usr/bin/env python3
"""
Script to reorganize the Testbed project structure.
Run this from the root of your Testbed directory.

Usage:
    python reorganize_project.py [--dry-run]
"""
import os
import shutil
import argparse
from pathlib import Path


def create_directory_structure(base_path: Path, dry_run: bool = False):
    """Create new directory structure."""
    
    dirs = [
        "testbed",
        "testbed/core",
        "testbed/hardware",
        "testbed/config",
        "testbed/simulators",
        "examples",
        "examples/tuning",
        "config/camera",
        "assets/markers",
        "data/trajectories",
        "data/results",
        "videos",
        "docs",
        "tests",
    ]
    
    print("Creating directory structure...")
    for dir_name in dirs:
        dir_path = base_path / dir_name
        if dry_run:
            print(f"  [DRY RUN] Would create: {dir_path}")
        else:
            dir_path.mkdir(parents=True, exist_ok=True)
            print(f"  ✓ Created: {dir_path}")
    
    # Create __init__.py files
    init_dirs = [
        "testbed",
        "testbed/core",
        "testbed/hardware",
        "testbed/config",
        "testbed/simulators",
        "examples",
        "tests",
    ]
    
    for dir_name in init_dirs:
        init_file = base_path / dir_name / "__init__.py"
        if not dry_run:
            init_file.touch()
            print(f"  ✓ Created: {init_file}")


def move_files(base_path: Path, dry_run: bool = False):
    """Move files to new locations."""
    
    moves = [
        # Core files
        ("testbed.py", "testbed/core/base_testbed.py"),
        
        # Hardware
        ("testbed_real.py", "testbed/real_testbed.py"),
        
        # Simulators
        ("testbed_virt.py", "testbed/simulators/virtual.py"),
        ("plotlab.py", "testbed/simulators/plotlab.py"),
        
        # Examples
        ("thesis_example.py", "examples/basic_simulation.py"),
        ("control_example.py", "examples/control_example.py"),
        ("tune_pid.py", "examples/tuning/tune_pid.py"),
        ("tune_pid_regression.py", "examples/tuning/tune_pid_regression.py"),
        ("example.py", "examples/simple_example.py"),
        ("data_gen.py", "examples/data_generation.py"),
        
        # Config
        ("Camera/cameraMatrix.txt", "config/camera/cameraMatrix.txt"),
        ("Camera/cameraDistortion.txt", "config/camera/cameraDistortion.txt"),
        
        # Assets
        ("Markers", "assets/markers"),
        
        # Data
        ("data/*.mat", "data/trajectories/"),
        ("data/*.csv", "data/results/"),
        
        # Videos
        ("Videos", "videos"),
        
        # Keep utilities as is
        # ("utilities", "testbed/controllers") - Already good
    ]
    
    print("\nMoving files...")
    for src, dst in moves:
        src_path = base_path / src
        dst_path = base_path / dst
        
        # Handle wildcards
        if '*' in src:
            import glob
            for file in glob.glob(str(src_path)):
                file_path = Path(file)
                dest = dst_path / file_path.name
                if dry_run:
                    print(f"  [DRY RUN] Would move: {file} -> {dest}")
                else:
                    if file_path.exists():
                        dest.parent.mkdir(parents=True, exist_ok=True)
                        shutil.move(str(file_path), str(dest))
                        print(f"  ✓ Moved: {file} -> {dest}")
        else:
            if dry_run:
                print(f"  [DRY RUN] Would move: {src} -> {dst}")
            else:
                if src_path.exists():
                    dst_path.parent.mkdir(parents=True, exist_ok=True)
                    if src_path.is_dir():
                        shutil.move(str(src_path), str(dst_path))
                    else:
                        shutil.copy2(str(src_path), str(dst_path))
                    print(f"  ✓ Moved: {src} -> {dst}")


def create_new_files(base_path: Path, dry_run: bool = False):
    """Create new configuration and documentation files."""
    
    files = {
        ".gitignore": """# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/

# Data
*.mat
*.csv
*.avi
*.mp4

# IDE
.vscode/
*.code-workspace
.idea/

# OS
.DS_Store
Thumbs.db

# Jupyter
*.ipynb
.ipynb_checkpoints/
""",
        
        "setup.py": """from setuptools import setup, find_packages

setup(
    name="testbed-robots",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        'numpy>=1.22.0',
        'matplotlib>=3.5.0',
        'opencv-contrib-python>=4.5.0',
        'scipy>=1.8.0',
        'pandas>=1.4.0',
        'pyserial>=3.5',
        'cvxopt>=1.2.0',
    ],
    python_requires='>=3.10',
    author="Your Name",
    description="Multi-robot testbed platform",
)
""",
        
        "docs/installation.md": """# Installation Guide

## Prerequisites

- Python 3.10+
- USB camera (for real hardware)
- ESP8266 WiFi modules
- ArUco markers (4x4_100 dictionary)

## Setup

1. Clone repository
2. Create virtual environment
3. Install dependencies: `pip install -r requirements.txt`
4. Configure camera calibration in `config/camera/`
5. Adjust serial ports in configuration

## Hardware Setup

See README.md for hardware connection details.
""",
        
        "docs/usage.md": """# Usage Guide

## Virtual Simulation

```python
from testbed.simulators.virtual import VirtualTestbed
import numpy as np

testbed = VirtualTestbed(number_of_robots=3)
# ... control loop
```

## Real Hardware

```python
from testbed.real_testbed import RealTestbed

testbed = RealTestbed(number_of_robots=3)
# ... control loop
```

See `examples/` for complete examples.
""",
    }
    
    print("\nCreating new files...")
    for filename, content in files.items():
        file_path = base_path / filename
        if dry_run:
            print(f"  [DRY RUN] Would create: {file_path}")
        else:
            file_path.parent.mkdir(parents=True, exist_ok=True)
            with open(file_path, 'w') as f:
                f.write(content)
            print(f"  ✓ Created: {file_path}")


def cleanup(base_path: Path, dry_run: bool = False):
    """Remove old/unnecessary files."""
    
    to_remove = [
        "__pycache__",
        "*.pyc",
        "Untitled.ipynb",
        "Testbed.code-workspace",
        "virtual.csv",
        "virtual2.csv",
        "virtual3.csv",
        "Camera",  # After moving contents
    ]
    
    print("\nCleaning up...")
    for pattern in to_remove:
        if '*' in pattern:
            import glob
            for file in glob.glob(str(base_path / pattern)):
                if dry_run:
                    print(f"  [DRY RUN] Would remove: {file}")
                else:
                    Path(file).unlink()
                    print(f"  ✓ Removed: {file}")
        else:
            path = base_path / pattern
            if path.exists():
                if dry_run:
                    print(f"  [DRY RUN] Would remove: {path}")
                else:
                    if path.is_dir():
                        shutil.rmtree(path)
                    else:
                        path.unlink()
                    print(f"  ✓ Removed: {path}")


def main():
    parser = argparse.ArgumentParser(description="Reorganize Testbed project")
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be done without making changes')
    args = parser.parse_args()
    
    base_path = Path.cwd()
    
    print("="*60)
    print("Testbed Project Reorganization")
    print("="*60)
    
    if args.dry_run:
        print("\n⚠️  DRY RUN MODE - No changes will be made\n")
    
    # Confirm
    if not args.dry_run:
        response = input("This will reorganize your project. Continue? (yes/no): ")
        if response.lower() != 'yes':
            print("Aborted.")
            return
    
    # Execute reorganization
    try:
        create_directory_structure(base_path, args.dry_run)
        move_files(base_path, args.dry_run)
        create_new_files(base_path, args.dry_run)
        cleanup(base_path, args.dry_run)
        
        print("\n" + "="*60)
        if args.dry_run:
            print("DRY RUN COMPLETE - No changes were made")
        else:
            print("✓ REORGANIZATION COMPLETE!")
        print("="*60)
        
        if not args.dry_run:
            print("\nNext steps:")
            print("1. Update imports in your scripts")
            print("2. Test virtual simulation: python examples/basic_simulation.py")
            print("3. Review docs/ for updated documentation")
    
    except Exception as e:
        print(f"\n❌ Error during reorganization: {e}")
        print("Project may be in an inconsistent state.")
        print("Consider restoring from backup.")


if __name__ == "__main__":
    main()
