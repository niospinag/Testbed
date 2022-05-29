# from re import A
from testbed_real import Testbed
import utilities.misc as misc 
import utilities.barrier_certificates as brct
import utilities.controllers as ctrl

import sys 

import numpy as np
import time

# Instantiate Robotarium object
N = 6
x = np.array([])
load_position, _ = misc.load_data_matlab('myData2.mat', frac_data = 3)
load_position(0)
# x = np.append(x, np.transpose([3, 1]))
# # print()
# print(x)
# x = np.append(x, [4, 1])
# # print()
# print(x)
# cache = {'int_err_v': np.zeros(N), 'int_err_w':np.zeros(N)}
# cache['int_err_v'][1] += 1
# print(cache['int_err_v'][1] )   