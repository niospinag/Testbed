'''this is an example of how to implement the textbed
'''
from testbed_real import Testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl

import sys 

import numpy as np
import time

# Instantiate Robotarium object data 
N = 12
sim_name = f'nrestricted_{N}v_7N' #name of the simulation
split_data = 7 # how many points would you like to split the data
#creates a function where give a specific point in the path
load_position = misc.load_data_matlab('data/' + sim_name+ '.mat' , \
     split_data = split_data,shift_x=-200, scale_x=1.3, shift_y=-85, scale_y=30) 


iteration = 0
initial_conditions = load_position(iteration)

# initial_conditions = misc.generate_initial_conditions(N)
r = Testbed(number_of_robots=N, show_figure = True, initial_conditions=initial_conditions)

# Define goal points by removing orientation from poses
goal_points = load_position(iteration)

# function to record video of camera
r.record_video('222sin_points2_10frac__' + sim_name)

# Create controller to implement in the experiment
unicycle_pose_controller = ctrl.create_pid_unicycle_pose_controller(linear_gain = [6, 0, 0], angular_gain = [17, 0.1 , 0.5], num_robots = N)

''' it is very important to call first 'get_poses' before call 'step' '''
x = r.get_poses()
r.step()

# funtion to draw the goals in the screen
r.draw_point(goal_points)

try:
    # Get poses of agents
    x = r.get_poses()
    
    # get dynamic target
    goal_points = load_position(iteration)
    
    r.draw_point(goal_points)
    
    # Create unicycle control inputs
    dxu = unicycle_pose_controller(x, goal_points) #, cache)

    # Set the velocities
    r.set_velocities(np.arange(N), dxu)
    
    # Iterate the simulation
    r.step()
    iteration +=1
except:
    # Close all connections and show the entire report
    r.call_at_scripts_end()