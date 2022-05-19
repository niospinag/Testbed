'''this is an example of how to implement the textbed
'''
from testbed_real import Testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl

import sys 

import numpy as np
import time

# Instantiate Robotarium object
N = 6

load_position, _ = misc.load_data_matlab('myData.mat', frac_data = 10)

initial_conditions = load_position(0)
# initial_conditions = misc.generate_initial_conditions(N)
r = Testbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = load_position(0)
print('goals', goal_points)
print('goals shape', goal_points.shape)
# Create unicycle pose controller
unicycle_pose_controller = ctrl.create_clf_unicycle_pose_controller(10, 2, 3)

# Create barrier certificates to avoid collision
uni_barrier_cert = brct.create_unicycle_barrier_certificate2()
print(r.poses)
# define x initially
x = r.get_poses()
r.step()

# r.draw_point(goal_points)
# While the number of robots at the required poses is less
# than N...

iteration = 0
try:
# if True:
    # while (np.size(misc.at_pose(x, goal_points, 200, 20) ) != N):
    while True:

        # Get poses of agents
        x = r.get_poses()
        
        # Create unicycle control inputs
        print(int(np.floor(iteration/10)))
        # goal_points = load_position(int(np.floor(iteration/10)))
        goal_points = load_position(iteration)
        
        r.draw_point(goal_points)
        dxu = unicycle_pose_controller(x, goal_points)
        
        # Create safe control inputs (i.e., no collisions)
        # dxu = uni_barrier_cert(dxu, x)

        # Set the velocities
        r.set_velocities(np.arange(N), dxu)

        # Iterate the simulation
        r.step()
        iteration +=1
except Exception as e:
    #Call at end of script to print debug information and for your script to run on the Robotarium server properly
    
    print("\033[1;31;40m  Error on line {}   \033[0m  ".format(sys.exc_info()[-1].tb_lineno))
    print(e)
    r.call_at_scripts_end()

finally:
    r.call_at_scripts_end()