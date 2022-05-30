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
N = 12
sim_name = f'nrestricted_{N}v_7N'
load_position, _ = misc.load_data_matlab('data/'+sim_name+ '.mat' , frac_data = 10)

iteration = 0 # 300
initial_conditions = load_position(iteration)
# initial_conditions = misc.generate_initial_conditions(N)
r = Testbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = load_position(iteration)

r.record_video(sim_name)

# Create unicycle pose controller
# unicycle_pose_controller = ctrl.create_clf_unicycle_position_controller(linear_velocity_gain=10, angular_velocity_gain=0.4)
unicycle_pose_controller = ctrl.create_pid_unicycle_pose_controller(linear_gain = [6, 0, 0], angular_gain = [10, 0.1 , 0.5], num_robots = N)
# unicycle_pose_controller = ctrl.create_reactive_pose_controlle(linear_gain = [5, 0, 0.1], angular_gain = [10, 0 , 0.1], num_robots = N)

# Create barrier certificates to avoid collision
# uni_barrier_cert = brct.create_unicycle_barrier_certificate2()
# define x initially
x = r.get_poses()
r.step()

# r.draw_point(goal_points)
# While the number of robots at the required poses is less
# than N..f.

# try:
if True:
    # cache = {'int_err_v': np.zeros(N), 'int_err_w': np.zeros(N), \
    #          'rate_err_v': np.zeros(N), 'rate_err_w': np.zeros(N), \
    #              'last_err_v': np.zeros(N), 'last_err_w': np.zeros(N), 'prev_time': time.time() }

    # while (np.size(misc.at_pose(x, goal_points, 200, 20) ) != N):
    while True:

        # Get poses of agents
        x = r.get_poses()
        
        # Create unicycle control inputs
        # print(int(np.floor(iteration/10)))
        # goal_points = load_position(int(np.floor(iteration/10)))
        goal_points = load_position(iteration)
        # print('g_p',goal_points)
        r.draw_point(goal_points)
        # dxu = unicycle_pose_controller(x, goal_points[:2,:])
        dxu = unicycle_pose_controller(x, goal_points) #, cache)
        # Create safe control inputs (i.e., no collisions)
        # dxu = uni_barrier_cert(dxu, x)

        # Set the velocities
        r.set_velocities(np.arange(N), dxu)
        
        # Iterate the simulation
        r.step()
        iteration +=1

r.call_at_scripts_end()