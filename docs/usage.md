# Usage Guide

## Virtual Simulation

```python
from testbed import VirtualTestbed

env = VirtualTestbed(number_of_robots=3)
# ... control loop
```

## Real Hardware

```python
from testbed.hardware.real import RealTestbed

env = RealTestbed(number_of_robots=3)
# ... control loop
```

## Decentralized Multi-Robot Control (Threaded)

Use one thread per robot to compute local control commands on the PC, then send the consolidated velocity matrix over WiFi/serial.

```python
from testbed.control.decentralized import DecentralizedController

# controller_fn(robot_id, poses, goals) -> np.array([v, w])
decentralized = DecentralizedController(number_of_robots=N, controller_fn=controller_fn)
commands = decentralized.compute(poses, goals)
env.set_velocities(range(N), commands)
```

See `examples/` for complete examples.
