from setuptools import setup, find_packages

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
