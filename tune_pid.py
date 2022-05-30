'''this is an example of how to implement the textbed
'''
import pandas as pd
import tqdm

from testbed_real import Testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl


import numpy as np
import time

# Instantiate Robotarium object
# N = 2

# load_position, _ = misc.load_data_matlab('myData.mat', frac_data = 10)

initial_conditions = np.array([[100 , -100, 100] , [40, -40, 0], [np.pi/2, -np.pi/2, 0]])
# initial_conditions = np.array([[100 ] , [40], [np.pi/2]])

N = initial_conditions.shape[1]


print(initial_conditions.shape)
# print(np.ones((3,2)))
# initial_conditions = misc.generate_initial_conditions(N)
r = Testbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = initial_conditions* [[-1], [1], [-1]]
# Create unicycle pose controller
data = {}

for kp in np.arange(10, 17, 1):
    for ki in np.arange(0, 2, 0.5):

# for kv_i in range(20):
#     for kw_i in range(4):

        print(f'datas used {kp} and {ki}')
        unicycle_pose_controller = ctrl.create_pid_unicycle_position_controller(linear_gain = [10, 0.01, 1], angular_gain = [kp, 0.2, ki], num_robots = N)
        # unicycle_pose_controller = ctrl.create_reactive_pose_controlle(linear_gain = [5, 0.1, 0.1], angular_gain = [10, 0 , 0.001], num_robots = N)
        x = r.get_poses()
        r.step()

        iteration = 0
        stay_time = 0
        # try:
        # if True:

        at_position = 0
        initial_time = time.time()
        while (at_position != N) or (stay_time < 10):
        # while True:
        # for _ in range(2):
            # Get poses of agents
            x = r.get_poses()
            
            # Create unicycle control inputs
            r.draw_point(goal_points)
            dxu = unicycle_pose_controller(x, goal_points)
            

            # Create safe control inputs (i.e., no collisions)
            
            # Set the velocities
            r.set_velocities(np.arange(N), dxu)
            
            # Iterate the simulation
            r.step()
            iteration +=1
            if at_position == N :
                stay_time += 1
            else:
                stay_time = 0

            if time.time()-initial_time > 30:
                break
            
            at_position = np.size(misc.at_pose(x, goal_points, position_error=10, rotation_error=0.3) )

        elapsed_time = time.time() - initial_time
        print(f'ki:{kp} y kd:{ki}')
        data[(kp, ki)] = [kp, ki, elapsed_time]
        print('next iteration')
        goal_points = goal_points* [[-1], [1], [-1]]
        


key_list = list(data.keys())
value_list = list(data.values())
min_value = min(value_list)
position = value_list.index(min_value)
print('min value', min_value)
print('key of minimum', key_list[position])

data_df = pd.DataFrame.from_dict(data, orient='index' , columns=[ 'ki', 'kd', 'time'])
data_df.to_csv('data/virtual1.csv')
print('mejores',data_df.sort_values('time', ascending = True))
print(data_df)
r.call_at_scripts_end()

#  ===================================================================
print(data_df)
flights = data_df.pivot("ki", "kd", "time")
print(flights)


import matplotlib.pyplot as plt
import seaborn as sns
f, ax = plt.subplots(figsize=(9, 6))
sns.heatmap(flights, annot=True, linewidths=.5, ax=ax)