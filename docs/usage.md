# Usage Guide

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
