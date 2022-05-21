'''this is an example of how to implement the textbed
'''
from testbed_real import Testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl


import numpy as np
import time

# Instantiate Robotarium object
N = 1

# load_position, _ = misc.load_data_matlab('myData.mat', frac_data = 10)

initial_conditions = np.array([[100] , [0], [np.pi/2]])

print(initial_conditions.shape)
print(np.ones((3,1)))
# initial_conditions = misc.generate_initial_conditions(N)
r = Testbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = np.array([[100] , [0], [np.pi/2]])
# Create unicycle pose controller
data = {}

for kv_i in range(4, 15):
    for kw_i in range(5, 15):
        goal_points = -goal_points
        unicycle_pose_controller = ctrl.create_pid_unicycle_pose_controlle(linear_gain = [kv_i, 0, 0], angular_gain = [kw_i, 1, 1], num_robots = N)
        x = r.get_poses()
        r.step()

        iteration = 0
        # try:
        # if True:
        cache = {'int_err_v': np.zeros(N), 'int_err_w': np.zeros(N), \
                'rate_err_v': np.zeros(N), 'rate_err_w': np.zeros(N), \
                    'last_err_v': np.zeros(N), 'last_err_w': np.zeros(N), 'prev_time': time.time() }
        initial_time = time.time()
        while (np.size(misc.at_pose(x, goal_points, 5, 5) ) != N):
        # while True:

            # Get poses of agents
            x = r.get_poses()
            
            # Create unicycle control inputs
            r.draw_point(goal_points)
            dxu, cache = unicycle_pose_controller(x, goal_points, cache)
            # Create safe control inputs (i.e., no collisions)
            
            # Set the velocities
            r.set_velocities(np.arange(N), dxu)
            
            # Iterate the simulation
            r.step()
            iteration +=1
        elapsed_time = time.time() - initial_time
        data[(kv_i, kw_i)] = elapsed_time
        print('next iteration')


key_list = list(data.keys())
value_list = list(data.values())
min_value = min(value_list)
position = value_list.index(min_value)
print('min value', min_value)
print('key of the minimum', key_list[position])
print(data)

r.call_at_scripts_end()