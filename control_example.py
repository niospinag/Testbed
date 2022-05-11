'''this is an example of how to implement the textbed
'''
import testbed as testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl

# import rps.robotarium as robotarium
# from rps.utilities.transformations import *
# from rps.utilities.barrier_certificates import *
# from rps.utilities.misc import *
# from rps.utilities.controllers import *

import numpy as np
import time

# Instantiate Robotarium object
N = 5
# initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))
initial_conditions = misc.generate_initial_conditions(N)
r = testbed.Testbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = misc.generate_initial_conditions(N)
print(goal_points)
# Create unicycle pose controller
unicycle_pose_controller = ctrl.create_clf_unicycle_pose_controller()

# Create barrier certificates to avoid collision
uni_barrier_cert = brct.create_unicycle_barrier_certificate()

# define x initially
x = r.get_poses()

r.step()

# While the number of robots at the required poses is less
# than N...
while (np.size(misc.at_pose(x, goal_points)) != N):

    # Get poses of agents
    x = r.get_poses()
    # print(x)
    # Create unicycle control inputs
    dxu = unicycle_pose_controller(x, goal_points)

    # Create safe control inputs (i.e., no collisions)
    dxu = uni_barrier_cert(dxu, x)
    # print(dxu)
    # Set the velocities
    r.set_velocities(np.arange(N), dxu)

    # Iterate the simulation
    r.step()

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
