'''
This is an example of how to implement the virtual testbed
'''
import sys
import time
from pathlib import Path
import numpy as np

# --- PATH CONFIGURATION ---
# Add the project root to sys.path to see the 'testbed' package
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# --- UPDATED IMPORTS ---
# 1. Import the main class from the package
from testbed import VirtualTestbed

# 2. Import the separated utilities (formerly misc)
from testbed.utils import io          # For loading .mat files
from testbed.utils import geometry    # For checking positions (at_pose)

# 3. Import the controllers
import testbed.control.controllers as ctrl

# --- SIMULATION CONFIGURATION ---
N = 3                   # Number of robots
data_name = 'data_7v_7N' # Name of the data file
split_data = 10         # Interpolation factor for points

# Define the correct path to the .mat file (based on your tree it is in data/trajectories)
# file_path = f'data/trajectories/{data_name}.mat'
file_path = project_root / 'data' / 'trajectories' / f'{data_name}.mat'
file_path = str(file_path)


# Load data (Using 'io' module instead of 'misc')
print(f"Loading trajectory from: {file_path}")
load_position = io.load_data_matlab(file_path, split_data=split_data, shift_x=0, scale_x=1, shift_y=0, scale_y=1)

# Initial conditions
iteration = 0
initial_conditions = load_position(iteration)[:, :N]

# --- TESTBED INITIALIZATION ---
# Use VirtualTestbed instead of generic Testbed
print("Initializing virtual simulator...")
r = VirtualTestbed(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions)

# Define initial goal points
goal_points = load_position(iteration)[:, :N]

# Create controller (PID)
unicycle_pose_controller = ctrl.create_pid_unicycle_pose_controller(
    linear_gain=[6, 0, 0], 
    angular_gain=[17, 0.1, 0.5], 
    num_robots=N
)

# --- MAIN LOOP ---
print("Starting control loop...")

# It is important to call 'get_poses' before the first 'step'
x = r.get_poses()
r.step()

# Draw initial goals on screen
r.draw_point(goal_points)

try:
    while True:
        # Use geometry.at_pose instead of misc.at_pose
        # Check if all robots have reached their goals
        while (np.size(geometry.at_pose(x, goal_points, position_error=10, rotation_error=4)) != N):
            
            # 1. Get current poses
            x = r.get_poses()
            
            # 2. Update goal visualization
            r.draw_point(goal_points)
            
            # 3. Calculate control law
            dxu = unicycle_pose_controller(x, goal_points)

            # 4. Send velocities to the simulator
            r.set_velocities(np.arange(N), dxu)
            
            # 5. Step the simulation
            r.step()
            
            # Optional: Small sleep to avoid CPU saturation if running too fast
            # time.sleep(0.01) 

        # If targets reached, move to the next point in the trajectory
        iteration += 1
        try:
            goal_points = load_position(int(iteration))[:, :N]
        except IndexError:
            print("End of trajectory reached.")
            break
        
except KeyboardInterrupt:
    print("\nSimulation stopped by user.")

except Exception as e:
    # Catch real errors to see them in console
    print(f"\n An error occurred: {e}")
    import traceback
    traceback.print_exc()

finally:
    # Close connections and show report
    r.call_at_scripts_end()