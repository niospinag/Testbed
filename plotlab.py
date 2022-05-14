import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import matplotlib.lines as lines
import matplotlib as mpl

mpl.rcParams['figure.dpi'] = 200


# plt.figure(figsize = (8,6), dpi = 80)
class Plotlab():
    def __init__(self, number_of_robots=1, show_figure=True, initial_conditions=np.array([]), time_step=0.2,
                 xf_pos=[], yf_pos=[]):
        # Check user input types
        # if yf_pos is None:
        #     yf_pos = []
        # if xf_pos is None:
        #     xf_pos = []
        assert isinstance(number_of_robots,int), "The number of robots used argument (number_of_robots) provided to create the Robotarium object must be an integer type. Recieved type %r." % type(number_of_robots).__name__
        assert isinstance(initial_conditions,np.ndarray), "The initial conditions array argument (initial_conditions) provided to create the Robotarium object must be a numpy ndarray. Recieved type %r." % type(initial_conditions).__name__
        assert isinstance(show_figure,bool), "The display figure window argument (show_figure) provided to create the Robotarium object must be boolean type. Recieved type %r." % type(show_figure).__name__

        # Check user input ranges/sizes
        assert (number_of_robots >= 0 and number_of_robots <= 50), "Requested %r robots to be used when creating the Robotarium object. The deployed number of robots must be between 0 and 50." % number_of_robots
        if (initial_conditions.size > 0):
            assert initial_conditions.shape == (3,number_of_robots), "Initial conditions provided when creating the Robotarium object must of size 3xN, where N is the number of robots used. Expected a 3 x %r array but recieved a %r x %r array."   % (number_of_robots, initial_conditions.shape[0], initial_conditions.shape[1])

        if isinstance(xf_pos,np.ndarray) and isinstance(yf_pos,np.ndarray):
            self.draw_path = True
        else:
            self.draw_path = False



        self.number_of_robots = number_of_robots
        self.show_figure = show_figure
        self.initial_conditions = initial_conditions

        # Boundary stuff -> lower left point / width / height
        # self.boundaries = [left_shift, down_shift, width, height]
        # self.boundaries = [-0, -1, 4, 2]
        self.boundaries = [-200, -150, 400, 300]

        self.file_path = None
        self.current_file_size = 0

        # Constants
        self.time_step = time_step
        self.wheel_radius = 3
        self.base_length = 7.071
        # self.heigth = 70.71
        self.robot_diameter = self.base_length*np.sqrt(2)
        self.wheel_distance = self.base_length*0.4
        self.color_robot = ['b', 'r', 'g', 'c', 'm', 'y', 'blue', 'orange', 'brown', 'purple', 'indigo', 'royalblue', 'pink', 'olive', 'green', 'lime', 'darkviolet', 'gold', 'silver', 'gray']

        # self.max_linear_velocity = 0.2
        # self.max_angular_velocity = 2 * (self.wheel_radius / self.robot_diameter) * (
        #             self.max_linear_velocity / self.wheel_radius)
        # self.max_wheel_velocity = self.max_linear_velocity / self.wheel_radius

        self.robot_radius = self.robot_diameter / 2

        # self.velocities = np.zeros((2, number_of_robots))
        self.poses = self.initial_conditions

        self.left_led_commands = []
        self.right_led_commands = []

        # Visualization
        self.figure = []
        self.axes = []
        # self.left_led_patches = []
        # self.right_led_patches = []
        self.chassis_patches = []
        self.right_wheel_patches = []
        self.left_wheel_patches = []
        self.future_position = []
        self.center_wheel_patches = []

        self.x_future_pos = xf_pos
        self.y_future_pos = yf_pos

        # Initialize some rendering variables
        self.previous_render_time = time.time()
        self.sim_in_real_time = True

        # self.figure, self.axes = plt.subplots(figsize=(self.boundaries[2], self.boundaries[3]))
        self.figure, self.axes = plt.subplots(figsize = (8,4))

        for i in range(self.number_of_robots):
            f_pos = lines.Line2D(self.x_future_pos, self.y_future_pos, alpha=0.3, ls= '--', color = self.color_robot[i], marker='.')
            p = patches.RegularPolygon(self.poses[:2, i], 4, math.sqrt(2) * self.robot_radius,
                                       self.poses[2, i] + math.pi / 4, facecolor=self.color_robot[i], edgecolor='k', label = "Vh "+ str(i))
            lw = patches.Circle(self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i] + math.pi / 2), np.sin(self.poses[2, i] + math.pi / 2)))\
                                + self.wheel_distance * np.array((-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2))) \
                                , self.wheel_radius, facecolor='k')

            rw = patches.Circle(self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i] - math.pi / 2), np.sin(self.poses[2, i] - math.pi / 2)))\
                                + self.wheel_distance * np.array((-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))
                                , self.wheel_radius, facecolor='k')

            cw = patches.Circle(self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))\
                                + self.wheel_distance * np.array((-np.sin(self.poses[2, i]), np.cos(self.poses[2, i])))
                                , 0.4*self.wheel_radius, facecolor='k')

            self.future_position.append(f_pos)
            self.right_wheel_patches.append(rw)
            self.left_wheel_patches.append(lw)
            self.center_wheel_patches.append(cw)
            self.chassis_patches.append(p)
            
            if self.draw_path:
                self.axes.add_line(f_pos)
            self.axes.add_patch(lw)
            self.axes.add_patch(rw)
            self.axes.add_patch(p)
            self.axes.add_patch(cw)
            
            
            # self.axes.plot([0, 0.1, 0.2, 0.3], [0, 0.7, 0.3, 0.5], 'o', linestyle="--", color= 'b')


            # Draw arena
        self.boundary_patch = self.axes.add_patch(patches.Rectangle(self.boundaries[:2], self.boundaries[2], self.boundaries[3], fill=False))

        self.axes.set_xlim(self.boundaries[0] - 0.1, self.boundaries[0] + self.boundaries[2] + 0.1)
        self.axes.set_ylim(self.boundaries[1] - 0.1, self.boundaries[1] + self.boundaries[3] + 0.1)
        self.axes.set_ylabel('Distance [m]')
        self.axes.set_title('Simulation')
        self.axes.grid(True, linestyle = '--', alpha=0.3)
        self.axes.legend()

        plt.ion()
        # plt.axis('equal')
        plt.show()

        # plt.subplots_adjust(left=-0.03, right=1.03, bottom=-0.03, top=1.03, wspace=0, hspace=0)

    def step(self, poses, xf_pos=[], yf_pos=[]):
        """plot each step of the simulation"""
        # if xf_pos is None:
        #     xf_pos = []
        # if yf_pos is None:
        #     yf_pos = []
        assert isinstance(poses,np.ndarray), "The initial conditions array argument (initial_conditions) provided to create the Robotarium object must be a numpy ndarray. Recieved type %r." % type(poses).__name__
        self.poses = poses
        # Update graphics
        # plt.cla()
        if self.show_figure:
            # if (self.sim_in_real_time):
            #     t = time.time()
            #     while (t - self.previous_render_time < self.time_step):
            #         t = time.time()
            #     self.previous_render_time = t

            for i in range(self.number_of_robots):

                if self.draw_path:
                    self.future_position[i].set_xdata(xf_pos[:,i])
                    self.future_position[i].set_ydata(yf_pos[:,i])

                self.right_wheel_patches[i].center = self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i] - math.pi / 2), np.sin(self.poses[2, i] - math.pi / 2))) + \
                                                     self.wheel_distance * np.array((-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))
                self.right_wheel_patches[i].orientation = self.poses[2, i] + math.pi / 4

                self.left_wheel_patches[i].center = self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i] + math.pi / 2), np.sin(self.poses[2, i] + math.pi / 2))) + \
                                                    self.wheel_distance * np.array((-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))
                self.left_wheel_patches[i].orientation = self.poses[2, i] + math.pi / 4

                self.center_wheel_patches[i].center = self.poses[:2, i] + self.robot_radius * np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i]))) + \
                                                    self.wheel_distance * np.array((-np.sin(self.poses[2, i] + math.pi / 2), np.cos(self.poses[2, i] + math.pi / 2)))


                self.chassis_patches[i].xy = self.poses[:2, i]
                self.chassis_patches[i].orientation = self.poses[2, i] + math.pi / 4

            self.figure.canvas.draw_idle()
            self.figure.canvas.flush_events()


if __name__ == "__main__":
    import scipy.io as spio

    mat = spio.loadmat('myData.mat', squeeze_me=True)


    def position(i):
        pos = np.array([hist_pos[:, i], zhist[:, i], np.zeros((6))])
        return pos

    def get_pos(i):
        # xphist[ f_estados, agente]
        nv = vphist.shape[2] #6
        hp = vphist.shape[1] #6
        fs = vphist.shape[0] # 30
        xphist = np.zeros((hp, nv))
        xphist[0,:]=position(i)[0]


        for j in range(hp-1):
            xphist[j+1,:] = xphist[j,:] + T* vphist[i, j, :]

        return xphist


    vhist = mat['vhist']  # structures need [()]
    vphist = mat['vphist']
    hist_pos = mat['hist_pos']
    zhist = mat['zhist']*10
    zphist = mat['zphist']*10

    T = mat['T']

    # pos = np.array([hist_pos[:,1], zhist[:,1], np.zeros((6))])

    visual = Plotlab(number_of_robots=6, show_figure=True, initial_conditions=position(0), xf_pos = get_pos(0), yf_pos= zphist[0,:,:])

    for i in range(30):
        poses = position(i)
        xphist = get_pos(i)
        visual.step(poses, xphist , zphist[i,:,:])
