#!/usr/bin/env python

# Subscribe to the odom topic to get the turtlebot odometry readings  
from ssl import OP_ENABLE_MIDDLEBOX_COMPAT
import rospy
import copy
import numpy as np
import matplotlib.pyplot as plt
#import open3d as o3d
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
#from tf.msg import tfMessage
from scipy.spatial.transform import Rotation
    
#################################################################################
# Subscriber callback function for odom saves data
def odom_callback(data):

    # Global variables for odom data
    global odom_pose, odom_twist

    # Save the data from the odometry readings
    odom_pose = data.pose
    odom_twist = data.twist

    #print(type(odom_pose))
    #plt.plot(odom_pose.pose.position.x, odom_pose.pose.position.y)
    #plt.show()
    # Test that the data is correctly being subscribed to
    #print(odom_pose.pose.position.x)

    # Add the current x and y pose positions to the odom lists
    odomX.append(odom_pose.pose.position.x)
    odomY.append(odom_pose.pose.position.y)

#################################################################################
# Subscriber callback function for the laser scan saves data
def laser_callback(data):

    # Global variables for odom data
    global laser_ranges, laser_intensities

    # Save the data from the odometry readings
    laser_ranges = data.ranges
    laser_intensities = data.intensities


#################################################################################
# Subscriber callback function for tf saves data
def tf_callback(data):

    # Global variables for odom data
    global tf_transform

    # Save the data from the odometry readings
    tf_transform = data.transforms.transform

    # Can see frame relationship between Odom and laser here
    # IN TERMINAL run: rostopic echo tf --> to see the frames
    #             run: rosrun tf tf_echo /frame1 /frame2 --> to see transform from frame 1 to frame 2

#################################################################################
# Function converts the input 3x3 rotation matrix into quaternions
def rotation_to_quaternion(rot_matrix):

    # Call the scipy function to convert to quaternions
    quats = Rotation.from_matrix(rot_matrix)
    return quats

    # Note: scipy can also convert euler angles, rotation vectors

#################################################################################
# Function sourced from Open3D Tutorials - visualises the alignment of the points
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    #o3d.visualization.draw_geometries([source_temp, target_temp])

#################################################################################
# Function sourced from Open3D Tutorials - calculates the resultant transformation
# between point clouds using point to point ICP registration algorithm
"""def icp_registration():

    # Get the source and target pointclouds from the LIDAR sensor - CHANGE TO GET SOURCE AT ONE 
    # INSTANCE AND TARGET LIKE 0.5 SECONDS LATER - SO CAN COMPARE TRANSFORMATION BETWEEN ROBOT MOVEMENT
    source = [1, 2, 3]
    target = [2, 3, 4]

    # Threshold for ICP correspondences
    threshold = 0.2

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    # Wait for user enter to visualise the points with initial estimate
    print("\nPress Enter to visualise points")
    raw_input()
    draw_registration_result(source, target, init_trans)

    # Evaluate ICP performance metrics - fitness, inlier RMSE and set size
    print("Evaluation of initial alignment")
    evaluation = o3d.registration.evaluate_registration(
        source, target, threshold, init_trans)
    print(evaluation)

    # Calculate transformation
    print("\nTransformation from point-to-point ICP")
    #reg_p2p = o3d.registration.registration_icp(
       # source, target, threshold, init_trans,
       # o3d.registration.TransformationEstimationPointToPoint(),
       # o3d.registration.ICPConvergenceCriteria(max_iteration=20000))
    print(reg_p2p)
    print("\nTransformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)"""""

#################################################################################
def main():

    # Global x and y pose list variables
    global odomX
    global odomY
    odomX = []
    odomY = []

    try:
        # Initialise a new node
        rospy.init_node('odom_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            # Subscribe to odometry, scan and tf topics
            rospy.Subscriber("/odom", Odometry, odom_callback)
            #rospy.Subscriber("/scan", LaserScan, laser_callback)
            #rospy.Subscriber("/tf", tfMessage, tf_callback)

            # Calls the ICP function
            #icp_registration()
            
            # Sleep until next spin
            rate.sleep()

        # Plot the odometry trajectory 
        plt.plot(odomX, odomY)
        plt.title("Odometry Readings Trajectory")
        plt.xlabel("X pose position")
        plt.ylabel("Y pose position")
        plt.show()


    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return
   
if __name__ == '__main__':
    main()