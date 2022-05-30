import numpy as np
import math
from utilities.transformations import *
import time

def create_si_position_controller(x_velocity_gain=10, y_velocity_gain=10, velocity_magnitude_limit=0.15):
    """Creates a position controller for single integrators.  Drives a single integrator to a point
    using a proportional controller.

    x_velocity_gain - the gain impacting the x (horizontal) velocity of the single integrator
    y_velocity_gain - the gain impacting the y (vertical) velocity of the single integrator
    velocity_magnitude_limit - the maximum magnitude of the produce velocity vector (should be less than the max linear speed of the platform)

    -> function
    """

    #Check user input types
    assert isinstance(x_velocity_gain, (int, float)), "In the function create_si_position_controller, the x linear velocity gain (x_velocity_gain) must be an integer or float. Recieved type %r." % type(x_velocity_gain).__name__
    assert isinstance(y_velocity_gain, (int, float)), "In the function create_si_position_controller, the y linear velocity gain (y_velocity_gain) must be an integer or float. Recieved type %r." % type(y_velocity_gain).__name__
    assert isinstance(velocity_magnitude_limit, (int, float)), "In the function create_si_position_controller, the velocity magnitude limit (y_velocity_gain) must be an integer or float. Recieved type %r." % type(y_velocity_gain).__name__
    
    #Check user input ranges/sizes
    assert x_velocity_gain > 0, "In the function create_si_position_controller, the x linear velocity gain (x_velocity_gain) must be positive. Recieved %r." % x_velocity_gain
    assert y_velocity_gain > 0, "In the function create_si_position_controller, the y linear velocity gain (y_velocity_gain) must be positive. Recieved %r." % y_velocity_gain
    assert velocity_magnitude_limit >= 0, "In the function create_si_position_controller, the velocity magnitude limit (velocity_magnitude_limit) must not be negative. Recieved %r." % velocity_magnitude_limit
    
    gain = np.diag([x_velocity_gain, y_velocity_gain])

    def si_position_controller(xi, positions):

        """
        xi: 2xN numpy array (of single-integrator states of the robots)
        points: 2xN numpy array (of desired points each robot should achieve)

        -> 2xN numpy array (of single-integrator control inputs)

        """

        #Check user input types
        assert isinstance(xi, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the single-integrator robot states (xi) must be a numpy array. Recieved type %r." % type(xi).__name__
        assert isinstance(positions, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the robot goal points (positions) must be a numpy array. Recieved type %r." % type(positions).__name__

        #Check user input ranges/sizes
        assert xi.shape[0] == 2, "\n \033[1;33;40m In the si_position_controller function created by the create_si_position_controller function, the dimension of the single-integrator robot states (xi) must be 2 ([x;y]). Recieved dimension %r.     \033[0m  \n" % xi.shape[0]
        assert positions.shape[0] == 2, "In the si_position_controller function created by the create_si_position_controller function, the dimension of the robot goal points (positions) must be 2 ([x_goal;y_goal]). Recieved dimension %r." % positions.shape[0]
        assert xi.shape[1] == positions.shape[1], "In the si_position_controller function created by the create_si_position_controller function, the number of single-integrator robot states (xi) must be equal to the number of robot goal points (positions). Recieved a single integrator current position input array of size %r x %r and desired position array of size %r x %r." % (xi.shape[0], xi.shape[1], positions.shape[0], positions.shape[1])

        _,N = np.shape(xi)
        dxi = np.zeros((2, N))

        # Calculate control input
        dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
        dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])

        # Threshold magnitude
        norms = np.linalg.norm(dxi, axis=0)
        idxs = np.where(norms > velocity_magnitude_limit)
        if norms[idxs].size != 0:
            dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

        return dxi

    return si_position_controller

def create_clf_unicycle_position_controller(linear_velocity_gain=0.8, angular_velocity_gain=3):
    """Creates a unicycle model pose controller.  Drives the unicycle model to a given position
    and orientation. (($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2 \times N}$)

    linear_velocity_gain - the gain impacting the produced unicycle linear velocity
    angular_velocity_gain - the gain impacting the produced unicycle angular velocity
    
    -> function
    """

    #Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), "In the function create_clf_unicycle_position_controller, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type %r." % type(linear_velocity_gain).__name__
    assert isinstance(angular_velocity_gain, (int, float)), "In the function create_clf_unicycle_position_controller, the angular velocity gain (angular_velocity_gain) must be an integer or float. Recieved type %r." % type(angular_velocity_gain).__name__
    
    #Check user input ranges/sizes
    assert linear_velocity_gain >= 0, "In the function create_clf_unicycle_position_controller, the linear velocity gain (linear_velocity_gain) must be greater than or equal to zero. Recieved %r." % linear_velocity_gain
    assert angular_velocity_gain >= 0, "In the function create_clf_unicycle_position_controller, the angular velocity gain (angular_velocity_gain) must be greater than or equal to zero. Recieved %r." % angular_velocity_gain
     

    def position_uni_clf_controller(states, positions):

        """  A position controller for unicycle models.  This utilized a control lyapunov function
        (CLF) to drive a unicycle system to a desired position. This function operates on unicycle
        states and desired positions to return a unicycle velocity command vector.

        states: 3xN numpy array (of unicycle states, [x;y;theta])
        poses: 3xN numpy array (of desired positons, [x_goal;y_goal])

        -> 2xN numpy array (of unicycle control inputs)
        """

        #Check user input types
        assert isinstance(states, np.ndarray), "In the function created by the create_clf_unicycle_position_controller function, the single-integrator robot states (xi) must be a numpy array. Recieved type %r." % type(states).__name__
        assert isinstance(positions, np.ndarray), "In the function created by the create_clf_unicycle_position_controller function, the robot goal points (positions) must be a numpy array. Recieved type %r." % type(positions).__name__

        #Check user input ranges/sizes
        assert states.shape[0] == 3, "In the function created by the create_clf_unicycle_position_controller function, the dimension of the unicycle robot states (states) must be 3 ([x;y;theta]). Recieved dimension %r." % states.shape[0]
        assert positions.shape[0] == 2, "In the function created by the create_clf_unicycle_position_controller function, the dimension of the robot goal positions (positions) must be 2 ([x_goal;y_goal]). Recieved dimension %r." % positions.shape[0]
        assert states.shape[1] == positions.shape[1], "In the function created by the create_clf_unicycle_position_controller function, the number of unicycle robot states (states) must be equal to the number of robot goal positions (positions). Recieved a current robot pose input array (states) of size %r states %r and desired position array (positions) of size %r states %r." % (states.shape[0], states.shape[1], positions.shape[0], positions.shape[1])


        _,N = np.shape(states)
        dxu = np.zeros((2, N))

        pos_error = positions - states[:2][:]
        rot_error = np.arctan2(pos_error[1][:],pos_error[0][:])
        dist = np.linalg.norm(pos_error, axis=0)

        dxu[0][:]=linear_velocity_gain*dist*np.cos(rot_error-states[2][:])
        dxu[1][:]=angular_velocity_gain*dist*np.sin(rot_error-states[2][:])

        return dxu

    return position_uni_clf_controller

def create_clf_unicycle_pose_controller(approach_angle_gain=1, desired_angle_gain=2.7, rotation_error_gain=1):
    """Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$) 
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This control is based on a control
    Lyapunov function.

    approach_angle_gain - affects how the unicycle approaches the desired position
    desired_angle_gain - affects how the unicycle approaches the desired angle
    rotation_error_gain - affects how quickly the unicycle corrects rotation errors.


    -> function
    """

    gamma = approach_angle_gain
    k = desired_angle_gain
    h = rotation_error_gain

    def R(theta):
        return np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])

    def pose_uni_clf_controller(states, poses):

        N_states = states.shape[1]
        dxu = np.zeros((2,N_states))

        for i in range(N_states):
            translate = R(-poses[2,i]).dot((poses[:2,i]-states[:2,i]))
            e = np.linalg.norm(translate)
            theta = np.arctan2(translate[1],translate[0])
            alpha = theta - (states[2,i]-poses[2,i])
            alpha = np.arctan2(np.sin(alpha),np.cos(alpha))

            ca = np.cos(alpha)
            sa = np.sin(alpha)

            print(gamma)
            print(e)
            print(ca)

            dxu[0,i] = gamma* e* ca
            dxu[1,i] = k*alpha + gamma*((ca*sa)/alpha)*(alpha + h*theta)

        return dxu

    return pose_uni_clf_controller

def create_hybrid_unicycle_pose_controller(linear_velocity_gain=1, angular_velocity_gain=2, velocity_magnitude_limit=0.15, angular_velocity_limit=np.pi, position_error=0.05, position_epsilon=0.02, rotation_error=0.05):
    '''Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$)
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This controller is
    based on a hybrid controller that will drive the robot in a straight line to the desired position then rotate
    to the desired position.
    
    linear_velocity_gain - affects how much the linear velocity is scaled based on the position error
    angular_velocity_gain - affects how much the angular velocity is scaled based on the heading error
    velocity_magnitude_limit - threshold for the max linear velocity that will be achieved by the robot
    angular_velocity_limit - threshold for the max rotational velocity that will be achieved by the robot
    position_error - the error tolerance for the final position of the robot
    position_epsilon - the amount of translational distance that is allowed by the rotation before correcting position again.
    rotation_error - the error tolerance for the final orientation of the robot

    '''

    si_to_uni_dyn = create_si_to_uni_dynamics(linear_velocity_gain=linear_velocity_gain, angular_velocity_limit=angular_velocity_limit)

    def pose_uni_hybrid_controller(states, poses, input_approach_state = np.empty([0,0])):
        N=states.shape[1]
        dxu = np.zeros((2,N))

        #This is essentially a hack since default arguments are evaluated only once we will mutate it with each call
        if input_approach_state.shape[1] != N: 
            approach_state = np.ones((1,N))[0]

        for i in range(N):
            wrapped = poses[2,i] - states[2,i]
            wrapped = np.arctan2(np.sin(wrapped),np.cos(wrapped))

            dxi = poses[:2,[i]] - states[:2,[i]]

            #Normalize Vector
            norm_ = np.linalg.norm(dxi)

            if(norm_ > (position_error - position_epsilon) and approach_state[i]):
                if(norm_ > velocity_magnitude_limit):
                    dxi = velocity_magnitude_limit*dxi/norm_
                dxu[:,[i]] = si_to_uni_dyn(dxi, states[:,[i]])
            elif(np.absolute(wrapped) > rotation_error):
                approach_state[i] = 0
                if(norm_ > position_error):
                    approach_state = 1
                dxu[0,i] = 0
                dxu[1,i] = angular_velocity_gain*wrapped
            else:
                dxu[:,[i]] = np.zeros((2,1))

        return dxu

    return pose_uni_hybrid_controller


    
def create_pid_unicycle_position_controller(linear_gain = [15, 0, 0], angular_gain = [16, 0, 0], num_robots = 1):
    """Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$) 
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This control is based on a PID controller.
    -> function
    """

    kp_v, ki_v, kd_v = linear_gain
    kp_w, ki_w, kd_w = angular_gain

    # cache = {'int_err_v': np.zeros(num_robots), 'int_err_w': np.zeros(num_robots), \
    #          'rate_err_v': np.zeros(num_robots), 'rate_err_w': np.zeros(num_robots), \
    #              'last_err_v': np.zeros(num_robots), 'last_err_w': np.zeros(num_robots), 'prev_time': time.time() }

    integralErrorV = np.zeros(num_robots)
    integralErrorW = np.zeros(num_robots)
    rate_err_v = np.zeros(num_robots)
    rate_err_w = np.zeros(num_robots)
    last_err_v = np.zeros(num_robots)
    last_err_w = np.zeros(num_robots)

    prev_time = time.time()


    def control_PID(states, poses): # , cache = cache):
        ''' states: 3xN numpy array (of unicycle states, [x;y;theta])
            poses: 3xN numpy array (of desired positons, [x_goal;y_goal ; tetha_goal])

            -> 2xN numpy array (of unicycle control inputs)
        '''

        # global prev_time
        N_states = states.shape[1]
        dxu = np.zeros((2,N_states))
        now = time.time()
        # dt = now - prev_time
        dt = 0.2
        # print('dt' , dt)
        # prev_time = 1*now
        for i in range(N_states):
        # correction of the angle
            targ_angle = math.atan2(poses[1,i]-states[1,i] , poses[0,i]-states[0,i])
            
            err_w = targ_angle - states[2,i] # error between the robot and target
            err_v = np.sqrt((poses[0,i] - states[0,i])**2 + (poses[1,i] - states[1,i])**2)
            # e_dist = np.linalg.norm(states[:2,i] - poses[:2,i])
                    

            if(err_w< -np.pi):
                err_w=2*np.pi+ err_w
            if(err_w > np.pi):
                err_w = -2*np.pi + err_w
            
            
            # if(np.abs(err_w)<(0.4)): #0.2
            #     err_w = 0
            
            if(err_v < 10):
                err_w = poses[2,i] - states[2,i]
                err_v=0
                integralErrorV[i]=0
                # last_err_v[i]=0
            
            integralErrorV[i] += err_v*dt
            integralErrorW[i] += err_w*dt
            rate_err_v[i] = ( err_v - last_err_v[i] )/dt
            rate_err_w[i] = ( err_w - last_err_w[i] )/dt

            
            
            
            # antibounding
            integralErrorV[i]= min(integralErrorV[i] , 45)
            integralErrorW[i]= min(integralErrorW[i] , 35)
            last_err_v[i] = err_v
            last_err_w[i] = err_w

            dxu[0,i] = kp_v*err_v + ki_v* integralErrorV[i] + kd_v * rate_err_v[i] 
            dxu[1,i] = kp_w*err_w + ki_w* integralErrorW[i] + kd_w * rate_err_w[i]


        return  dxu
        

    return control_PID


def create_pid_unicycle_pose_controller(linear_gain = [15, 0, 0], angular_gain = [16, 0, 0], num_robots = 1):
    """Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$) 
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This control is based on a PID controller.
    -> function
    """

    kp_v, ki_v, kd_v = linear_gain
    kp_w, ki_w, kd_w = angular_gain

    integralErrorV = np.zeros(num_robots)
    integralErrorW = np.zeros(num_robots)
    rate_err_v = np.zeros(num_robots)
    rate_err_w = np.zeros(num_robots)
    last_err_v = np.zeros(num_robots)
    last_err_w = np.zeros(num_robots)

    prev_time = time.time()


    def control_PID(states, poses): # , cache = cache):
        ''' states: 3xN numpy array (of unicycle states, [x;y;theta])
            poses: 3xN numpy array (of desired positons, [x_goal;y_goal ; tetha_goal])

            -> 2xN numpy array (of unicycle control inputs)
        '''

        
        N_states = states.shape[1]
        dxu = np.zeros((2,N_states))

        now = time.time()
        # dt = now - prev_time
        # prev_time = now
        dt = 0.2
        for i in range(N_states):
        # correction of the angle
            targ_angle = math.atan2(poses[1,i]-states[1,i] , poses[0,i]-states[0,i])
            
            err_w = targ_angle - states[2,i] # error between the robot and target
            err_v = np.sqrt((poses[0,i] - states[0,i])**2 + (poses[1,i] - states[1,i])**2)
            # e_dist = np.linalg.norm(states[:2,i] - poses[:2,i])
        
            if(err_w< -np.pi):
                err_w=2*np.pi+ err_w
            if(err_w > np.pi):
                err_w = -2*np.pi + err_w
            
            
            # if(np.abs(err_w)<(0.4)): #0.2
            #     err_w = 0
            
            
            integralErrorV[i] += err_v*dt
            integralErrorW[i] += err_w*dt
            rate_err_v[i] = ( err_v - last_err_v[i] )/dt
            rate_err_w[i] = ( err_w - last_err_w[i] )/dt

            
            if(abs(err_v)<7):
                err_w=0
                integralErrorW[i] = 0
            
            # antibounding
            integralErrorV[i]= min(integralErrorV[i] , 45)
            integralErrorW[i]= min(integralErrorW[i] , 35)
            last_err_v[i] = err_v
            last_err_w[i] = err_w

            dxu[0,i] = kp_v*err_v + ki_v* integralErrorV[i] + kd_v * rate_err_v[i] 
            dxu[1,i] = kp_w*err_w + ki_w* integralErrorW[i] + kd_w * rate_err_w[i]




        return  dxu
        

    return control_PID


def create_reactive_pose_controller(linear_gain = [15, 0, 0], angular_gain = [16, 0, 0], num_robots = 1):
    """Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$) 
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This control is based on a PID controller.

    approach_angle_gain - affects how the unicycle approaches the desired position
    desired_angle_gain - affects how the unicycle approaches the desired angle
    rotation_error_gain - affects how quickly the unicycle corrects rotation errors.


    -> function
    """
    controller_pid = create_pid_unicycle_position_controller(linear_gain = linear_gain, angular_gain = angular_gain, num_robots = num_robots)



    def find_nearest_robot(states, id):

        distances = np.transpose(states[:2,:]) - states[:2,id] 
        array = np.linalg.norm(distances,axis=1)
        array = np.asarray(array)
        near_value = np.abs(array - array[id])
        near_value[id] += 1000
        idx = (near_value).argmin()
        # vector_norm = distances[idx]/near_value[id]
        return states[:2,idx], idx


    def control_poses(states, poses): # , cache = cache):
            ''' states: 3xN numpy array (of unicycle states, [x;y;theta])
                poses: 3xN numpy array (of desired positons, [x_goal;y_goal ; tetha_goal])

                -> 2xN numpy array (of unicycle control inputs)
            '''
            # print('states array', states)
            # print('poses array', poses)

            
            N_states = states.shape[1]
            new_goal = np.zeros((3,N_states))
            dxu = np.zeros((2,N_states))

            # now = time.time()
            # dt = now - prev_time

            for i in range(N_states):
                if (states[:2,i] - poses[:2,i]).any():
                    #find the closest neightbors
                    
                    
                    state_rb, idx = find_nearest_robot(states, i)
                    vect_obs = state_rb - states[:2, i] 
                    # print('vect_obs', vect_obs)
                    vect_obs_norm = vect_obs/ np.linalg.norm(vect_obs)
                    vect_goal = poses[:2,i]- states[:2,i]
                    vect_goal_norm = vect_goal/np.linalg.norm(vect_goal)
                    direction = 0.6* vect_goal_norm - 0.4*vect_obs_norm
                    
                    if  np.linalg.norm(vect_obs) < 15: #np.linalg.norm(vect_goal) > 10 or
                        
                        new_goal[:,i] = states[:,i]+np.append(direction*50, 0)
                    
                    else:
                        new_goal[:,i] = poses[:,i]                        

                    

                else:
                    print(f'the vehicle {i+1} is in the goal')

                # controller_pid(states, new_goal)

            return  controller_pid(states, new_goal)
    


    return control_poses