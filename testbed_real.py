
import time
import math
# from abc import ABC, abstractmethod
import plotlab as plb
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import serial

import utilities.misc as misc 
import utilities.controllers as ctrl
# vision libraries
import cv2
import cv2.aruco as aruco
import utilities.ArucoModule as cam


class Testbed():

    def __init__(self, number_of_robots=-1, show_figure=True, sim_in_real_time=True, initial_conditions=np.array([]), simulation= True):

        #Check user input types
        assert isinstance(number_of_robots,int), "The number of robots used argument (number_of_robots) provided to create the Testbed object must be an integer type. Recieved type %r." % type(number_of_robots).__name__
        assert isinstance(initial_conditions,np.ndarray), "The initial conditions array argument (initial_conditions) provided to create the Testbed object must be a numpy ndarray. Recieved type %r." % type(initial_conditions).__name__
        assert isinstance(show_figure,bool), "The display figure window argument (show_figure) provided to create the Testbed object must be boolean type. Recieved type %r." % type(show_figure).__name__
        assert isinstance(sim_in_real_time,bool), "The simulation running at 0.033s per loop (sim_real_time) provided to create the Testbed object must be boolean type. Recieved type %r." % type(sim_in_real_time).__name__
        assert isinstance(simulation,bool), "true simulation shows a virtual implementation of the current experiment, false simulation implement the experiment in the real life %r." % type(simulation).__name__
        
        #Check user input ranges/sizes
        assert (number_of_robots >= 0 and number_of_robots <= 50), "Requested %r robots to be used when creating the Testbed object. The deployed number of robots must be between 0 and 50." % number_of_robots 
        
        self.number_of_robots = number_of_robots
        self.show_figure = show_figure
        self.initial_conditions = initial_conditions
        self.simulation = simulation
        self.goals = np.array([])

        # Boundary stuff -> lower left point / width / height
        self.boundaries = [-200, -150, 200, 150] # [-x -y x y]<===== check the dimentions of the testbed

        self.file_path = None
        self.current_file_size = 0

        # Constants
        self.time_step = 0.033
        self.robot_diameter = 20  #<===== check
        self.wheel_radius = 3  #<===== check
        self.base_length = 11  #<===== check
        self.max_linear_velocity = 200  #<===== check
        self.max_angular_velocity = 2*(self.wheel_radius/self.robot_diameter)*(self.max_linear_velocity/self.wheel_radius)  #<===== check
        self.max_wheel_velocity = self.max_linear_velocity/self.wheel_radius  #<===== check

        self.robot_radius = self.robot_diameter/2

        self.velocities = np.zeros((2, number_of_robots))
        self.poses = self.initial_conditions
        if self.initial_conditions.size == 0:
            self.poses = self.get_poses()
        
        self.left_led_commands = []
        self.right_led_commands = []

        self.controller = ctrl.create_clf_unicycle_position_controller(10, 1.5)        


        # self.visual = plb.Plotlab(number_of_robots=self.number_of_robots, show_figure=True, initial_conditions=self.initial_conditions, xf_pos = [], yf_pos= [])

        # # Visualization
        # self.figure = []
        # self.axes = []
        # self.left_led_patches = []
        # self.right_led_patches = []
        # self.chassis_patches = []
        # self.right_wheel_patches = []
        # self.left_wheel_patches = []

        # opencv parameters ///////////////////////////////////////////////////////
        exposure = -5
        self.WIDTH = 1280 # 1280 // 1920  //1600 //1024 //640
        self.HEIGHT = 720 # 720 // 1080  //896  // 576  //360

        self.cap = cv2.VideoCapture(0)
        self.img = None
        self.augDics = cam.loadAugImages("Markers")
        self.marker_size = 10.2  # - [cm]

        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self.camera_matrix, self.camera_distortion = cam.getCameraMatr("Camera", width=1280, height=720)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)  
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)  
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)

        self.position = np.ones((3, 100))
        self.detected = []
        self.d_points = False
        # initialize serial cominication
        self.esp8266 = serial.Serial("/dev/ttyUSB0", 115200)

        if (initial_conditions.size > 0):
            assert initial_conditions.shape == (3, number_of_robots), "Initial conditions provided when creating the Testbed object must of size 3xN, where N is the number of robots used. Expected a 3 x %r array but recieved a %r x %r array." % (number_of_robots, initial_conditions.shape[0], initial_conditions.shape[1])
            # Move the vehicles to the initial conditions decired 
            actual_pose = self.get_poses()
            self.move2target(actual_pose, self.initial_conditions)

        
        else:
            initial_conditions=self.get_poses()[:,:number_of_robots]
            assert initial_conditions.shape == (3, number_of_robots), "Camera does not detect enough vehicles. Please make sure you put the needed agents. Expected %r agents, but recieved %r." % (number_of_robots, initial_conditions.shape[1])


#Initialize some rendering variables
        self.previous_render_time = time.time()
        self.sim_in_real_time = sim_in_real_time

        #Initialize checks for step and get poses calls
        self._called_step_already = True
        self._checked_poses_already = False

        #Initialization of error collection.
        self._errors = {}

        #Initialize steps
        self._iterations = 0 


    def move2target(self, initial, final):
        x=initial
        N = self.number_of_robots
        print('atpose', misc.at_pose(x, final) )
        while (np.size(misc.at_pose(x, final) ) != N):
        
            # Create safe control inputs (i.e., no collisions)
            dxu = self.controller(x, final)

            dataControl = str(self.number_of_robots) + '\n'  # numero de marcadores
            r, g, b = (70, 40, 10)
            for id in range(self.number_of_robots):
                dataControl += "%0.0f; %0.1f; %0.1f; %0.1f; %0.1f; %0.1f" % (id+1, 1 * dxu[0,id], 1 * dxu[1,id], r, g, b) + '\n'
                
            dataControl += 'xxF'
            datos = dataControl.encode("utf-8")
            self.esp8266.write(datos)
            print(datos)


            x=self.get_poses()

            
        input('Press any key to start the implementation')

    def set_velocities(self, ids, velocities):
        # in case of problems ''' sudo chmod 666 /dev/ttyUSB0 '''
        pass

        # Threshold linear velocities
        idxs = np.where( np.abs(velocities[0, :]) > self.max_linear_velocity )
        velocities[0, idxs] = self.max_linear_velocity*np.sign( velocities[0, idxs] )

        # Threshold angular velocities
        idxs = np.where(np.abs(velocities[1, :]) > self.max_angular_velocity)
        velocities[1, idxs] = self.max_angular_velocity*np.sign(velocities[1, idxs])
        
        self.velocities = velocities

        #Protected Functions
    def _threshold(self, dxu):
        dxdd = self._uni_to_diff(dxu)

        to_thresh = np.absolute(dxdd) > self.max_wheel_velocity
        dxdd[to_thresh] = self.max_wheel_velocity*np.sign(dxdd[to_thresh])

        dxu = self._diff_to_uni(dxdd)

    def _uni_to_diff(self, dxu):
        r = self.wheel_radius
        l = self.base_length
        dxdd = np.vstack((1/(2*r)*(2*dxu[0,:]-l*dxu[1,:]),1/(2*r)*(2*dxu[0,:]+l*dxu[1,:])))

        return dxdd

    def _diff_to_uni(self, dxdd):
        r = self.wheel_radius
        l = self.base_length
        dxu = np.vstack((r/(2)*(dxdd[0,:]+dxdd[1,:]),r/l*(dxdd[1,:]-dxdd[0,:])))

        return dxu

    def _validate(self, errors = {}):
        # This is meant to be called on every iteration of step.
        # Checks to make sure robots are operating within the bounds of reality.

        p = self.poses
        b = self.boundaries
        N = self.number_of_robots


        for i in range(N):
            x = p[0,i]
            y = p[1,i]

            if(x < b[0] or x > (b[0] + b[2]) or y < b[1] or y > (b[1] + b[3])):
                    if "boundary" in errors:
                        errors["boundary"] += 1
                    else:
                        errors["boundary"] = 1
                        errors["boundary_string"] = "iteration(s) robots were outside the boundaries."

        for j in range(N-1):
            for k in range(j+1,N):
                if(np.linalg.norm(p[:2,j]-p[:2,k]) <= self.robot_diameter):
                    if "collision" in errors:
                        errors["collision"] += 1
                    else:
                        errors["collision"] = 1
                        errors["collision_string"] = "iteration(s) where robots collided."

        dxdd = self._uni_to_diff(self.velocities)
        exceeding = np.absolute(dxdd) > self.max_wheel_velocity
        if(np.any(exceeding)):
            if "actuator" in errors:
                errors["actuator"] += 1
            else:
                errors["actuator"] = 1
                errors["actuator_string"] = "iteration(s) where the actuator limits were exceeded."

        return errors


    def get_poses(self):
        """Returns the states of the agents.

        -> 3xN numpy array (of robot poses)
        """

        # assert(not self._checked_poses_already), "Can only call get_poses() once per call of step()."
        # Allow step() to be called again.
        self._called_step_already = False
        self._checked_poses_already = True 

        T1 = time.perf_counter()
        secuess, img = self.cap.read()
        self.img = img
        bboxs, ids = cam.findArucoMarkers(img)
        self.detected = ids
        print('ids', ids)
        # loop throug all the markers and augment each one
        if len(bboxs) != 0:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(bboxs, self.marker_size, self.camera_matrix,
                                                            self.camera_distortion)

            for n, (bbox, id) in enumerate(zip(bboxs, ids)):
                # draw augmented arucos

                if int(id) in self.augDics.keys():
                    img = cam.augmentAruco(bbox.astype(int), id, img, self.augDics[int(id)])
                # save position
                id = int(id)
                theta = misc.angle_correction(bbox)  # get the angle value
                self.poses[0, id-1] = tvec[n, 0, 0]
                self.poses[1, id-1] = tvec[n, 0, 1]
                self.poses[2, id-1] = theta

                str_position = "id=%0.0f x=%0.1f y=%0.1f theta=%0.2f" % (id, tvec[n, 0, 0], tvec[n, 0, 1], theta)
                cv2.putText(img, str_position, (0, 20 + 60 * id), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2,
                            cv2.LINE_AA)


        else:
            print('I DO NOT SEE ARUCOS!!!')

        # --- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # cv2.destroyAllWindows()
            self.call_at_scripts_end()
        
        cam.draw_axis(img, self.HEIGHT, self.WIDTH, color=(100, 100, 100))
        
        if self.d_points:
            for i in range(self.number_of_robots): 
                cam.draw_point(img, self.goals[:2,i], self.WIDTH, self.HEIGHT , color=(0, 200, 0))
               
        # --- Display the frame
        cv2.imshow("Image", img)


        return self.poses

    def call_at_scripts_end(self):
        """Call this function at the end of scripts to display potentail errors.  
        Even if you don't want to print the errors, calling this function at the
        end of your script will enable execution on the Testbed testbed.
        """
        

        print(' \033[1;32;40m ##### DEBUG OUTPUT #####   \033[0m ')
        print('\033[1;32;40m Your simulation will take approximately {0} real seconds when deployed on the Testbed.  \033[0m  \n'.format(math.ceil(self._iterations*0.033)))

        if bool(self._errors):
            if "boundary" in self._errors:
                print('\t \033[1;32;40m Simulation had {0} {1}  \033[0m \n'.format(self._errors["boundary"], self._errors["boundary_string"]))
            if "collision" in self._errors:
                print('\t \033[1;32;40m Simulation had {0} {1}  \033[0m \n'.format(self._errors["collision"], self._errors["collision_string"]))
            if "actuator" in self._errors:
                print('\t \033[1;32;40m Simulation had {0} {1}  \033[0m Simulation had {0} {1} \n'.format(self._errors["actuator"], self._errors["actuator_string"]))
        else:
            print('\033[1;32;40m No errors in your simulation! Acceptance of your experiment is likely!   \033[0m ')

        self.esp8266.close()
        self.cap.release()
        cv2.destroyAllWindows()
        exit()
        return

    def step(self):
        """Increments the simulation by updating the dynamics.
        """
        assert(not self._called_step_already), "Make sure to call get_poses before calling step() again."
        
        # Allow get_poses function to be called again.
        self._called_step_already = True
        self._checked_poses_already = False
        
        # Validate before thresholding velocities
        self._errors = self._validate()
        self._iterations += 1
        
        dataControl = str(self.number_of_robots)  # numero de marcadores
        dataControl += '\n'

        r, g, b = (0, 0, 10)
        for id in range(self.number_of_robots):
            dataControl += "%0.0f; %0.1f; %0.1f; %0.1f; %0.1f; %0.1f" % (id+1, 1 * self.velocities[0,id], 1 * self.velocities[1,id], r, g, b) + '\n'
            # dataControl += "%0.0f; %0.0f; %0.0f; %0.0f; %0.0f; %0.0f;" % (id, 1 * 200, 1 * 2, r, g, b) + '\n'
        
        # dataControl += "%0.0f; %0.0f; %0.0f; %0.0f; %0.0f; %0.0f;" % (nm, 1*vOutPut[nm-1],  1*wOutPut[nm-1], red, green, blue) + '\n'

        dataControl += 'xxF'
        datos = dataControl.encode("utf-8")
        self.esp8266.write(datos)
        print(datos)
        


    def draw_point(self,goals):
        assert isinstance(goals,np.ndarray), "The Goals array argument provided to show in screen the target must be a numpy ndarray. Recieved type %r." % type(initial_conditions).__name__
        # assert initial_conditions.shape == (3, number_of_robots), "Initial conditions provided when creating the Testbed object must of size 3xN, where N is the number of robots used. Expected a 3 x %r array but recieved a %r x %r array." % (number_of_robots, initial_conditions.shape[0], initial_conditions.shape[1])

        self.goals = goals
        self.d_points = True        
        
