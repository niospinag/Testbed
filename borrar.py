import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import matplotlib.lines as lines
import matplotlib as mpl
from abc import ABC, abstractmethod
import scipy.io as spio

mpl.rcParams['figure.dpi'] = 200

import rps.robotarium as robotarium

mat = spio.loadmat('myData.mat', squeeze_me=True)
vhist = mat['vhist']  # structures need [()]
vphist = mat['vphist']
hist_pos = mat['hist_pos']
zhist = mat['zhist']
zphist = mat['zphist']
T = mat['T']

def position(i):
    pos = np.array([hist_pos[:, i], zhist[:, i], np.zeros((6))])
    return pos/100


# figure, axes = plt.subplots(figsize=(6, 2))
# figure, axes = plt.subplots()
# plt.show()
robot_diameter = 0.11
number_of_robots = 6
robot_radius = robot_diameter / 2

boundaries = [-1.6, -1, 3.2, 2]
chassis_patches = []
left_led_patches = []
right_led_patches = []
right_wheel_patches  = []
left_wheel_patches = []

def plotting(poses):
    global axes
    # axes.set_axis_off()
    for i in range(number_of_robots):
        p = patches.RegularPolygon(poses[:2, i], 4, math.sqrt(2) * robot_radius,
                                   poses[2, i] + math.pi / 4, facecolor='#FFD700', edgecolor='k')
        rled = patches.Circle(poses[:2, i] + 0.75 * robot_radius * np.array(
            (np.cos(poses[2, i]), np.sin(poses[2, i])) + \
            0.04 * np.array((-np.sin(poses[2, i] + math.pi / 2), np.cos(poses[2, i] + math.pi / 2)))), \
                              robot_radius / 5, fill=False)
        lled = patches.Circle(poses[:2, i] + 0.75 * robot_radius * np.array(
            (np.cos(poses[2, i]), np.sin(poses[2, i])) + \
            0.015 * np.array((-np.sin(poses[2, i] + math.pi / 2), np.cos(poses[2, i] + math.pi / 2)))), \
                              robot_radius / 5, fill=False)
        rw = patches.Circle(poses[:2, i] + robot_radius * np.array(
            (np.cos(poses[2, i] + math.pi / 2), np.sin(poses[2, i] + math.pi / 2))) + \
                            0.04 * np.array(
            (-np.sin(poses[2, i] + math.pi / 2), np.cos(poses[2, i] + math.pi / 2))), \
                            0.02, facecolor='k')
        lw = patches.Circle(poses[:2, i] + robot_radius * np.array(
            (np.cos(poses[2, i] - math.pi / 2), np.sin(poses[2, i] - math.pi / 2))) + \
                            0.04 * np.array((-np.sin(poses[2, i] + math.pi / 2))), \
                            0.02, facecolor='k')
        # lw = patches.RegularPolygon(poses[:2, i]+robot_radius*np.array((np.cos(poses[2, i]-math.pi/2), np.sin(poses[2, i]-math.pi/2)))+\
        #                                0.035*np.array((-np.sin(poses[2, i]+math.pi/2), np.cos(poses[2, i]+math.pi/2))),\
        #                                4, math.sqrt(2)*0.02, poses[2,i]+math.pi/4, facecolor='k')

        chassis_patches.append(p)
        left_led_patches.append(lled)
        right_led_patches.append(rled)
        right_wheel_patches.append(rw)
        left_wheel_patches.append(lw)

        axes.add_patch(rw)
        axes.add_patch(lw)
        axes.add_patch(p)
        axes.add_patch(lled)
        axes.add_patch(rled)

    # Draw arena
    boundary_patch = axes.add_patch(patches.Rectangle(boundaries[:2], boundaries[2], boundaries[3], fill=False))

    axes.set_xlim(boundaries[0] - 0.1, boundaries[0] + boundaries[2] + 0.1)
    axes.set_ylim(boundaries[1] - 0.1, boundaries[1] + boundaries[3] + 0.1)

    # plt.ion()
    plt.show()

    # plt.subplots_adjust(left=-0.03, right=1.03, bottom=-0.03, top=1.03, wspace=0, hspace=0)


r  = robotarium.Robotarium(number_of_robots=6, show_figure=True, initial_conditions=position(0), sim_in_real_time=True)
for i in range(30):
    _= r.get_poses()
    r.poses = position(i)
    r.step()
    # plotting(position(1))
