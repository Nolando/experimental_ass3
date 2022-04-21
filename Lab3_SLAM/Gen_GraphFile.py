 #!/usr/bin/env python

'''
The file generates a graph file, which basicly stores all the edges(measurements) that have been received from the simulation.
If you run this file while moving the turtlebot around the simulator, it will save these measurements to a txt file.

The format of this file is as follow:
    ODOM3D [index] [odom_x]  [covariance_x] [odom_y] [covariance_y] [odom_theta] [covariance_theta] 
    MEAS2D [index] [landmark_id] [landmark_x] [covariance_y] [landmark_y] [covariance_y]
ODOM3D - Odometry Measurement
MEAS2D - Landmark Measurement

'''

import rospy
import numpy as np
import geometry

# Message types
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from landmarks_msg.msg import Landmarks_Msg

std_dev_linear_vel =  1
std_dev_y_change = 0.0001
std_dev_angular_vel =  (20 * np.pi) / 180


save_file = "graph.txt"


class GenGraphFile():
    

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        
        
        # Subscribe to the cylinder node output
        self.landmark_sub = rospy.Subscriber('/landmarks', Landmarks_Msg, self.callbackLandmarkMeasurement, queue_size=10000)

        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callbackOdometryMotion, queue_size=10000)
        

        # Initialise robot
        self.accum_odo = np.zeros((3,1))
        self.robot_poses = []

        self.landmarks = []
        self.time = None  # variable to keep current time
        self.seen_landmarks_labels = []
        self.meas = []
        self.meas_indices = []
        self.measurements = 0
        self.measurement_covariance = []

        f = open(save_file, "w")
        f.close()

        rospy.spin()
        
        
    def callbackOdometryMotion(self, msg):
        """
        This will be called every time we receive an odom reading
        """
        # Compute dtrobot_pose = duration of the sensor measurement
        if self.time is None:
            # handle the first measurement
            self.time = msg.header.stamp
            return
        dt = (msg.header.stamp.secs - self.time.secs) + (msg.header.stamp.nsecs - self.time.nsecs) * 10 ** (-9)
        self.time = msg.header.stamp

        # read linear and angular velocities from teh ROS message
        linear_vel = msg.twist.twist.linear.x  # velocity of the robot in its instantaneous frame. X is the forward direction
        angular_vel = msg.twist.twist.angular.z  # angular velocity about the Z axis of the robot's instantaneous frame

        # when there is no motion do not perform a prediction
        # if abs(linear_vel) < 0.009 and abs(angular_vel) < 0.09:
        #     return

        # Simulation: we have set s_linear_vel and s_angular_vel to be proportional to the distance or angle moved
        # In real robots this can be read directly from the odometry message if it is provided
        s_linear_vel_x = std_dev_linear_vel * linear_vel * dt  # 5 cm / seg
        s_linear_vel_y = 0.00001 # just a small value as there is no motion along y of the robot
        s_angular_vel = std_dev_angular_vel * angular_vel * dt  # 2 deg / seg

        # compute the motion command [dx, dy, dtheta]
        # Simulation: here we add the random gaussian noise proportional to the distance travelled and angle rotated.
        # Note: this is an approximation but works as time steps are small

        dx = linear_vel * dt + s_linear_vel_x * np.random.standard_normal()
        dy = 0.0000001*std_dev_y_change*np.random.standard_normal() #is no motion along y of the robot
        dtheta = angular_vel * dt + s_angular_vel * np.random.standard_normal()
        u = np.array([dx, dy, dtheta])

        motion_covariance_x = (s_linear_vel_x)**2
        motion_covariance_y = (s_linear_vel_y)**2
        motion_covariance_theta = (s_angular_vel)**2
        
        self.motion_command = u
        update_pose, _, _ = geometry.Relative2AbsolutePose(self.accum_odo.reshape(3,1), u.reshape(3,1))
        self.accum_odo = update_pose.reshape(3,1)
        

        # normalize angle
        self.accum_odo[2] = geometry.pi2pi(self.accum_odo[2])


        if np.abs(self.accum_odo[0]) > 0.02 or np.abs(self.accum_odo[1]) > 0.02 or np.abs(self.accum_odo[2]) > np.pi/180*10:            
            
            f = open(save_file, "a")
            line ='ODOM3D %s %.10f %.10f %.10f %.10f %.10f %.10f \n' % (self.measurements, 
                    self.accum_odo[0], motion_covariance_x, self.accum_odo[1], motion_covariance_y, self.accum_odo[2], motion_covariance_x)
            f.write(line)
            for i in range(len(self.meas_indices)):
                line = 'MEAS2D %s %s %.10f %.10f %.10f %.10f \n' % (self.measurements, self.meas_indices[i], self.meas[i][0], self.measurement_covariance[0], self.meas[i][1], self.measurement_covariance[1])
                f.write(line)

            self.measurements +=1
            print('Save Line')
            f.close()

            self.accum_odo = np.zeros((3,1))
            self.meas_indices = []
            self.meas = []



    def callbackLandmarkMeasurement(self, data):

        # For your cylinder detector, we'd suggest to use model state messages http://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelStates.html
        # the label (colour) can be set in the model name and the position in pose messages, then iterate over them to create the dictionary Landmarks_r
        # where the key is the label, you can store the position (x,y) in the robot reference frame, alternatively store the variance of the detection
        # Landmarks in robot frame
        print("Received {} landmark(s).".format(len(data.landmarks)))

        measurements = []
        indexes = []
        for landmark in data.landmarks:
            dx = landmark.x
            dy = landmark.y
            label = landmark.label

            if label not in self.seen_landmarks_labels:
                self.seen_landmarks_labels.append(label)

            measurements += [np.array([dx,dy]).T]
            indexes += [self.seen_landmarks_labels.index(label)]
        self.meas = measurements
        self.meas_indices = indexes

        self.measurement_covariance = [landmark.s_x, landmark.s_y]


        # Deal with measurement covariance close to zero
        if landmark.s_x < 10**(-4) and landmark.s_y < 10**(-4):
            print("Measurement covariance is close to zero.")
            self.measurement_covariance = [0.01, 0.01]

        
if __name__ == '__main__':
    print('Landmark SLAM Started...')
    # Initialize the node and name it.
    rospy.init_node('listener')
    # Go to class functions that do all the heavy lifting. Do error checking.
    mode = 1
    try:
        GenGraphFile()
    except rospy.ROSInterruptException: pass
