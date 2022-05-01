#!/usr/bin/env python
# Machine Unlearning

# Subscribe to the odom topic to get the turtlebot odometry readings  
import rospy
import copy
import numpy as np
import open3d as o3d
import tf                                       # /tf
from nav_msgs.msg import Odometry               # /odom
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan           # /scan
from tf.msg import tfMessage
from scipy.spatial.transform import Rotation

#################################################################################
# Global variables

# Open3d point clouds
new_pcd = o3d.geometry.PointCloud()
old_pcd = o3d.geometry.PointCloud()

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
    #print(odom_pose)

    # Add the current x and y pose positions to the odom lists
    odomX.append(odom_pose.pose.position.x)
    odomY.append(odom_pose.pose.position.y)


######################### Relative and Absolute Error ##################################
# Function to calculate the relative and absolute trajectory error 
def error(lidarDataX, lidarDataY, odomDataX, odomDataY, pointInteval):

    # Error lists for average calculation
    relativeErrorsX = []
    absoluteErrorsX = []
    relativeErrorsY = []
    absoluteErrorsY = []

    # Check if it is time to output the errors for the next interval
    if pointCount == pointInteval:

        # Error sum variables for a single interval
        totalRelativeX = 0
        totalAbsoluteX = 0
        totalRelativeY = 0
        totalAbsoluteY = 0

        # Calculate the sum of the relative and absolute errors
        for errorIndex in range(0, pointInteval):
            totalRelativeX += relativeErrorsX[errorIndex]
            totalRelativeY += relativeErrorsY[errorIndex]
            totalAbsoluteX += absoluteErrorsX[errorIndex]
            totalAbsoluteY += absoluteErrorsY[errorIndex]

        # Calculate average errors and display
        averageRelativeX = totalRelativeX/len(relativeErrorsX)
        averageRelativeY = totalRelativeY/len(relativeErrorsY)
        averageAbsoluteX = totalAbsoluteX/len(absoluteErrorsX)
        averageAbsoluteY = totalAbsoluteY/len(absoluteErrorsY)
        
    else:
        # Calculate the relative and absolute error for the current pair of points
        currentRelativeX = abs(odomDataX - lidarDataX)
        currentRelativeY = abs(odomDataY - lidarDataY)
        currentAbsoluteX = currentRelativeX/odomDataX
        currentAbsoluteY = currentRelativeY/odomDataY

        # Add the current errors to their respective lists
        relativeErrorsX.append(currentRelativeX)
        relativeErrorsY.append(currentRelativeY)
        absoluteErrorsX.append(currentAbsoluteX)
        absoluteErrorsY.append(currentAbsoluteY)

    return averageRelativeX, averageRelativeY, averageAbsoluteX, averageAbsoluteY

#################################################################################
# Subscriber callback function for the laser scan saves data
def laser_callback(data):

    # Update old pointcloud variable - NEED TO COPY OR ELSE IS PYTHON REFERENCES
    old_pcd = copy.deepcopy(new_pcd)

    # Global variables for odom data
    global laser_ranges

    # Save the ranges and intensities from the odometry readings
    laser_ranges = np.array([data.ranges], np.float64)

    # Empty row of zeroes for Vector3dVector - this seems to work better in calculating transformation
    zero_row = np.zeros(laser_ranges.size)

    # Create the n x 3 matrix for open3d type conversion
    new_data = np.vstack([laser_ranges, zero_row, zero_row])
    new_data = np.transpose(new_data)

    # Convert the numpy array to open3d type
    new_pcd.points = o3d.utility.Vector3dVector(new_data)

    # print(np.asarray(old_pcd.points))
    # print(np.asarray(new_pcd.points))

    # Check that the pointclouds are different
    if old_pcd.points != new_pcd.points and np.asarray(old_pcd.points).size != 0:
        
        # Conduct ICP registration
        icp_registration(old_pcd, new_pcd)

    # LATER IN Q1 CAN CHANGE THE MAX AND MIN RANGE BY WRITING TO RANGE_MIN AND RANGE_MAX

#################################################################################
# Subscriber callback function for tf saves data
def tf_callback(data):

    # Global variables for odom data
    global tf_transform

    # Save the data from the odometry readings
    tf_transform = data.transforms

    # print("\n\n-------------------------------")
    print(tf_transform)

    # Can see frame relationship between Odom and laser here
    # IN TERMINAL run: rostopic echo tf --> to see the frames
    #             run: rosrun tf tf_echo /frame1 /frame2 --> to see transform from frame 1 to frame 2

#################################################################################
# Function converts the input 3x3 rotation matrix into quaternions
def rotation_to_quaternion(rot_matrix):
    
    # Note: scipy can also convert euler angles, rotation vectors
    # Call the scipy function to convert to quaternions
    quats = Rotation.from_matrix(rot_matrix)
    return quats

#################################################################################
# Function sourced from Open3D Tutorials - visualises the alignment of the points
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

#################################################################################
# Function sourced from Open3D Tutorials - calculates the resultant transformation
# between point clouds using point to point ICP registration algorithm
# Source point cloud is the old, target point cloud is the new
def icp_registration(source, target):
        
    # Threshold for ICP correspondences
    threshold = 0.02

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    test = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, )

    # Calculate transformation
    # print("\n\n-----------------------\nTransformation from point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, init_trans,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=10000))
    # print(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)

#################################################################################
def main():

    # Global x and y pose list variables
    global odomX
    global odomY
    odomX = []
    odomY = []

    try:
        # Initialise a new node
        rospy.init_node('odom_ICP_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Can't estimate with ICP on first loop iteration
        ICP_bool = False

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            # print("\n-------------------------------\nloop city fam")

            # Subscribe to odometry, scan and tf topics
            # rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)
            rospy.Subscriber("/tf", tfMessage, tf_callback, queue_size=1)


            # Sleep until next spin
            rate.sleep()

        # Plot the odometry trajectory 
        plt.plot(odomX, odomY)
        plt.title("Odometry Readings Trajectory")
        plt.xlabel("X pose position")
        plt.ylabel("Y pose position")
        # plt.show()


    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return
   
if __name__ == '__main__':
    main()
