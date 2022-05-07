#!/usr/bin/env python

# from statistics import covariance
# from numpy import indices
# import rospy
import numpy as np
#Message types
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path, Odometry
# from landmarks_msg.msg import Landmarks_Msg
# from visualization_msgs.msg import Marker, MarkerArray

# from tf_conversions import transformations
# from std_msgs.msg import Float32

# Solver
from scipy.linalg import cho_factor, cho_solve

from Robot import Robot
from Landmark import Landmark

# Relative and Absolute conversions
import geometry

import matplotlib.pyplot as plt
import copy
import time

# Covariance/std to experiment with
std_dev_linear_vel =  0.01
std_dev_y_change = 0.0001
std_dev_angular_vel =  (5 * np.pi) / 180

landmark_measurement_covariance = np.array([[0.05,0.0],[0.0, 0.05]])
pose_measurement_covariance = np.array([[std_dev_linear_vel**2,0,0],[0,std_dev_y_change**2,0],[0,0,std_dev_angular_vel**2]])


# landmarks' most recent absolute coordinate
class LeastSquaresSolver(object):
    """
    This is the class that implements the Landmark SLAM Information Filter.
    Using a Nonlinear Least Squares Solver.
    """
    def __init__(self, threshold = 1e-12, max_num_iter = 1000, div_threshold = 1e2):
        """
        Initialize the object.
        """

        self.A = []
        self.b = []

        self.threshold = threshold
        self.max_num_iter = max_num_iter
        self.done = False
        self.list_prev_solutions = []
        self.prev_soln = None
        self.iters = 0
        self.div_threshold = div_threshold
        self.solver_start = None
        self.solver_finish = None



    def solve(self, states,landmarks, debug = True):
                
        self.states = states
        self.landmarks = landmarks

        self.iter = 0
        self.done = False

        # Helper values to describe the system for debugging
        num_states = len(self.states)
        num_landmarks = len(self.landmarks)
        num_measurements = self.get_num_measurements(num_states, num_landmarks)

        print("Preparing to solve a system with {} poses, {} landmarks and a total {} measurements between them.".format(num_states,num_landmarks,num_measurements))
        self.solver_start = time.time()
        

        while not self.done:
            
            # Construct and Update A and b matrix
            self.construct_system(num_states,num_landmarks,num_measurements)
            state_vec = self.construct_state_vec()          # Current robot poses, current landmarks 

            # Solve system
            delta_star = self.perform_cholesky_solve(self.A,self.b)

            self.update_vertices(delta_star, state_vec)

            # Check if error less than threshold, logging
            if debug and self.iter%2 == 0:
                print("Current Delta-Norm = " + str(np.linalg.norm(delta_star)) + " on iteration " + str(self.iter))
            if np.linalg.norm(delta_star) < self.threshold or self.iter > self.max_num_iter:
                print("Solved with error: {} on iteration # {}".format(np.linalg.norm(delta_star),self.iter))
                self.solver_finish = time.time()
                print("Time taken: {} seconds.".format(self.solver_finish-self.solver_start))
                self.done = True
            elif np.linalg.norm(delta_star) > self.div_threshold and self.iter > 0:
                print("SOLVER DIVERGED. Stopping...")
                self.solver_finish = time.time()
                print("Time taken: {} seconds.".format(self.solver_finish-self.solver_start))
                self.done = True
            
            self.iter += 1

        return self.states, self.landmarks
        
    # Performs solving of system
    def perform_cholesky_solve(self, A, b):

        left_hs = np.matmul(A.T, A)
        right_hs = np.matmul(-A.T, b)

        LLt, low = cho_factor(left_hs)

        return cho_solve((LLt,low), right_hs)

    # Calculate total number of measurements from states
    def get_num_measurements(self, num_states, num_landmarks):

        measurements = num_states * num_landmarks
        return measurements


    def construct_system(self,num_state,num_landmark, num_measurement):
        landmark = self.landmarks[0]

        '''
        A matrix:
        Num columns - n_states * 3 (for x, y, phi) + n_landmarks * 2 (for x,y)
        Num rows - n_states * 3 (for x, y, phi) + n_measurements * 2(???)
        '''

        # Initial previous pose is [0,0,0]
        prev_pose = np.array((0,0,0))
        prev_pose = np.reshape(prev_pose, (3,1))
        
        # Initialise A and b
        self.A = np.zeros((num_state*3+num_measurement*2,num_state*3+num_landmark*2))
        self.b = np.zeros(((num_state*3+num_measurement*2),1))

        current_H_col = 0

        # Construct the top portion of the A matrix, you will need to go through your robot poses carefully.
        # For each state
        for state in range(num_state):

            robot = self.states[state]

            # Predict the current pose given the previous pose (where curr_pose is an artificial x_i)
            # curr_pose = robot.predict_state(prev_pose)
            curr_odo = robot.get_odo()
            curr_pose,_,_ =  geometry.Relative2AbsolutePose(prev_pose, curr_odo)
            #curr_pose = np.reshape(3,1)
            # New landmark?

            # Calculate residual and Jacobians for motion
            predicted_motion, F_k, G_k = geometry.Absolute2RelativePose(prev_pose, curr_pose)
            residual_motion = predicted_motion - np.reshape(robot.get_odo(),(3,1))

            # Multiply by motion covariance
            F_k = np.multiply(F_k, robot.get_covariance_processed())
            G_k = np.multiply(G_k, robot.get_covariance_processed())
            residual_motion = np.matmul(robot.get_covariance_processed(),residual_motion)
            
            # Add to A matrix
            for row in range(len(G_k)):
                for col in range(len(G_k)):
                    self.A[state*3+row, state*3+col] = float(G_k[row,col])
                    if state != 0:
                        self.A[state*3+row,(state-1)*3+col] = float(F_k[row,col])

            # Add to b matrix 
            for row in range(len(residual_motion)):
                self.b[state*3+row] = float(residual_motion[row])


            # Error for the landmark measurements
            for landmk in (robot.get_landmarks()):
                landmark = self.landmarks[landmk]

                predicted_measurement, H_k, J_j = geometry.Absolute2RelativeXY(curr_pose[:,0], landmark.get_current()) 
                residual_measurement = predicted_measurement - robot.get_measurements()

                # Multiply by motion covariance
                H_k = np.matmul(H_k, robot.get_covariance_processed())
                J_j = np.matmul(J_j, robot.get_covariance_processed()[:2,:2])
                residual_measurement = np.matmul(robot.get_covariance_processed()[:2,:2], np.reshape(residual_measurement[:,0,0],(2,1)))

                for row in range(len(J_j)):
                    for col in range(len(J_j)):
                        self.A[num_state*3+state*num_landmark*2+landmk*2+row,num_state*3+landmk*2+col] = float(J_j[row,col])

                for row in range(len(H_k)):
                    for col in range(len(J_j)):
                        self.A[num_state*3+state*num_landmark*2+landmk*2+row,current_H_col*3+col] = float(H_k[row,col])

                for row in range(len(residual_measurement)):
                    self.b[num_state*3+state*num_landmark+landmk*2+row] = float(residual_motion[row])

            current_H_col += 1


    # Construct a vector of the state values
    def construct_state_vec(self):
        # This is a column vector. All poses on top of all landmarks
        # Total dimension = 3*num_poses + 2*num_landmarks

        state_vec = []
        state_dim = len(self.states)*3 + len(self.landmarks)*2

        for state in self.states:
            for value in state.get_odo():
                state_vec.append(value)
        
        for landmk in self.landmarks:
            for value in landmk.get_current():
                state_vec.append(value)

        return state_vec
        
    def update_vertices(self, delta_star, state_vec):

        new_state_vec = np.zeros(len(state_vec))
        for i in range(len(state_vec)):
            new_state_vec[i] = state_vec[i] + float(delta_star[i])

        for state_idx in range(len(self.states)):
            current = new_state_vec[state_idx*3:(state_idx+1)*3]
            self.states[state_idx].set_current(current)
        
        offset = len(self.states)*3
        for landmk_idx in range(len(self.landmarks)):
            current = new_state_vec[offset + landmk_idx*2 : offset+(landmk_idx+1)*2]
            self.landmarks[landmk_idx].set_current(current)
        



# class SLAM():
#     # Must have __init__(self) function for a class, similar to a C++ class constructor.
#     def __init__(self, mode):
        
#         modes = [1, 2]
#         assert (mode in modes),"Modes - 1: Batch, 2: Iterative"

#         self.pub_traj = rospy.Publisher('/robot_trajectory', Path, queue_size=5)
#         self.pub_landmarks = rospy.Publisher('/landmarks_state', MarkerArray, queue_size=5)

#         # Subscribe to the cylinder node output
#         self.landmark_sub = rospy.Subscriber('/landmarks', Landmarks_Msg, self.callbackLandmarkMeasurement, queue_size=10000)

#         # Subscribe to odometry
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callbackOdometryMotion, queue_size=10000)

#         # Set up viz timer
#         self.viz_timer = rospy.Timer(rospy.Duration(0.5), self.vizualizer_timer_cb)

        
#         # Initialise robot
#         self.solver = LeastSquaresSolver()
#         self.accum_odo = np.zeros((3,1))
#         self.robot_poses = []

#         self.landmarks = []
#         self.time = None  # variable to keep current time
#         self.seen_landmarks_labels = []
#         self.meas = []
#         self.meas_indices = []
#         self.skip = False
        
#         if mode == 1:
#             self.batch_slam(number_meas=20)
#         elif mode == 2:
#             self.batch_slam(number_meas=1)



#     def callbackOdometryMotion(self, msg):
#         """
#         This will be called every time we receive an odom reading
#         """
#         if self.skip:
#             return
#         # Compute dtrobot_pose = duration of the sensor measurement
#         if self.time is None:
#             # handle the first measurement
#             self.time = msg.header.stamp
#             return
#         dt = (msg.header.stamp.secs - self.time.secs) + (msg.header.stamp.nsecs - self.time.nsecs) * 10 ** (-9)
#         self.time = msg.header.stamp

#         # read linear and angular velocities from teh ROS message
#         linear_vel = msg.twist.twist.linear.x  # velocity of the robot in its instantaneous frame. X is the forward direction
#         angular_vel = msg.twist.twist.angular.z  # angular velocity about the Z axis of the robot's instantaneous frame

#         # when there is no motion do not perform a prediction
#         # if abs(linear_vel) < 0.009 and abs(angular_vel) < 0.09:
#         #     return

#         # Simulation: we have set s_linear_vel and s_angular_vel to be proportional to the distance or angle moved
#         # In real robots this can be read directly from the odometry message if it is provided
#         s_linear_vel_x = std_dev_linear_vel * linear_vel * dt  # 5 cm / seg
#         s_angular_vel = std_dev_angular_vel * angular_vel * dt  # 2 deg / seg

#         # compute the motion command [dx, dy, dtheta]
#         # Simulation: here we add the random gaussian noise proportional to the distance travelled and angle rotated.
#         # Note: this is an approximation but works as time steps are small

#         dx = linear_vel * dt + s_linear_vel_x * np.random.standard_normal()
#         dy = 0.0000001*std_dev_y_change*np.random.standard_normal() #is no motion along y of the robot
#         dtheta = angular_vel * dt + s_angular_vel * np.random.standard_normal()
#         curr_meas = copy.deepcopy(self.meas)
#         curr_meas_indices = copy.deepcopy(self.meas_indices)
#         print("Measured: {}".format(curr_meas))
#         # Calculate motion command and set it
#         u = np.array([dx, dy, dtheta])
        
#         self.motion_command = u


#         # This section is how we store the mesurements from the simulation to the datastructure
#         # This is a guide, depending on how you implemented the solver this might need to change.
#         update_pose, _, _ = geometry.Relative2AbsolutePose(self.accum_odo.reshape(3,1), u.reshape(3,1))
#         self.accum_odo = update_pose.reshape(3,1)
        

#         # normalize angle
#         self.accum_odo[2] = geometry.pi2pi(self.accum_odo[2])


#         if np.abs(self.accum_odo[0]) > 0.02 or np.abs(self.accum_odo[1]) > 0.02 or np.abs(self.accum_odo[2]) > np.pi/180*10:
#             #if no robot poses yets
            
#             if len(self.robot_poses)==0:
#                 position = np.zeros((3,1))
#             # Apply coordinate transform with the accumulated odo    
#             else:
#                 prev = copy.deepcopy(self.robot_poses[-1].get_current().reshape(3,1))
#                 # TODO Get the position of the robot in absolute ref, for initial guess of pose
#                 position = ...


#             self.robot_poses+= [Robot(position.reshape((3,1)),self.accum_odo.reshape(3,1),len(self.robot_poses),curr_meas,
#                                         curr_meas_indices, pose_measurement_covariance)]
#             self.accum_odo = np.zeros((3,1))

#             if len(self.seen_landmarks_labels) > len(self.landmarks):


#                 for i in range(len(self.landmarks),len(self.seen_landmarks_labels)):
                    
#                     # TODO Get estimate of landmark in absolute position from the measurment
#                     meas_location = ...
#                     self.landmarks += [Landmark(meas_location, i, landmark_measurement_covariance)]
                


#     def callbackLandmarkMeasurement(self, data):

#         # For your cylinder detector, we'd suggest to use model state messages http://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelStates.html
#         # the label (colour) can be set in the model name and the position in pose messages, then iterate over them to create the dictionary Landmarks_r
#         # where the key is the label, you can store the position (x,y) in the robot reference frame, alternatively store the variance of the detection
#         # Landmarks in robot frame
#         if self.skip:
#             return
#         print("Received {} landmark(s).".format(len(data.landmarks)))

#         measurements = []
#         indexes = []
#         for landmark in data.landmarks:
#             dx = landmark.x
#             dy = landmark.y
#             label = landmark.label

#             if label not in self.seen_landmarks_labels:
#                 self.seen_landmarks_labels.append(label)

#             measurements += [np.array([dx,dy]).T]
#             indexes += [self.seen_landmarks_labels.index(label)]
#         self.meas = measurements
#         self.meas_indices = indexes


#     # Publish the most uptodate estimate for the robot pose
#     def publish_trajectory_history(self, window_size = -1):
#         traj_msg = Path()
#         traj_msg.header.stamp = rospy.Time.now()
#         traj_msg.header.frame_id = "odom"

#         rospy.loginfo("Publishing {} robot poses".format(len(self.robot_poses)))
#         states = copy.deepcopy(self.robot_poses)
#         for robot in states:
#             pose = robot.get_current()
#             x = pose[0]
#             y = pose[1]
#             theta = pose[2]
#             pose = PoseStamped()
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             pose.pose.position.z = 0.0

#             quat = transformations.quaternion_from_euler(0.0, 0.0, theta)
#             pose.pose.orientation.x = quat[0]
#             pose.pose.orientation.y = quat[1]
#             pose.pose.orientation.z = quat[2]
#             pose.pose.orientation.w = quat[3]

#             traj_msg.poses.append(pose)

#             if window_size > 0 and len(traj_msg.poses) > window_size:
#                 traj_msg.poses.pop(0)

#         self.pub_traj.publish(traj_msg)


#     # Publishes landmark positions to be visible to rvis
#     def publish_landmark_state(self):
#         marker_array_msg = MarkerArray()
#         current = copy.deepcopy(self.landmarks)
#         for i, landmark in enumerate(current):
#             pose = landmark.get_current()
#             x = pose[0]
#             y = pose[1]
#             marker = Marker()
#             marker.header.frame_id = 'odom'
#             marker.id = self.seen_landmarks_labels[i]
#             marker.type = 3
#             marker.action = 0
#             marker.pose.position.x = x
#             marker.pose.position.y = y
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0
#             marker.scale.x = 0.1
#             marker.scale.y = 0.1
#             marker.scale.z = 0.1
#             marker.frame_locked = False
#             marker.lifetime = rospy.Duration(0.0)
#             marker_array_msg.markers.append(marker)

#         self.pub_landmarks.publish(marker_array_msg)


#     # This function is run every 0.5 seconds and publishes to topics
#     # Can visulise these in rvis
#     def vizualizer_timer_cb(self, timer_event):
#         rospy.loginfo_once("Started viz timer")
#         self.publish_trajectory_history()
#         self.publish_landmark_state()

    
#     # Run a single batch of measurements through the SLAM algorithm
#     def batch_slam(self,number_meas):
        
#         r = rospy.Rate(100)
#         while not rospy.is_shutdown():
#             if len(self.landmarks)>0 and len(self.robot_poses)>=number_meas and len(self.robot_poses[-1].landmarks)>0:
#                 self.skip = True

#                 # TODO Perform solve and plot results 
#                 # Becareful about what your passing, might need to copy varibles 

#                 break
            
#             r.sleep()
         

        
# if __name__ == '__main__':
#     print('Landmark SLAM Started...')
#     # Initialize the node and name it.
#     rospy.init_node('listener')
#     # Go to class functions that do all the heavy lifting. Do error checking.
#     mode = 1
#     try:
#         slam = SLAM(mode)
#     except rospy.ROSInterruptException: pass
