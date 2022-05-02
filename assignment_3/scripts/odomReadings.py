#!/usr/bin/env python3
# Machine Unlearning

# Subscribe to the odom topic to get the turtlebot odometry readings  
from numpy import dtype
import rospy
import copy
import numpy as np
import open3d as o3d

import laser_geometry.laser_geometry as lg
import ros_numpy

from nav_msgs.msg import Odometry               # /odom
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan           # /scan
from rospy_tutorials.msg import Floats


#################################################################################
# Subscriber callback function for odom saves data
def odom_callback(data):

    # Global variables for odom data
    global odom_pose, odom_twist

    # Save the data from the odometry readings
    odom_pose = data.pose
    odom_twist = data.twist

    # Add the current x and y pose positions to the odom lists
    odomX.append(np.float64(odom_pose.pose.position.x))                                         # float64
    odomY.append(np.float64(odom_pose.pose.position.y))                                         # float64

####################### Relative and Absolute Error #############################
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

    global new_data, laser_ranges

    # Update old pointcloud variable - NEED TO COPY OR ELSE IS PYTHON REFERENCES
    # old_pcd.points = copy.deepcopy(new_pcd.points)
    old_pcd = copy.deepcopy(new_pcd)

    # Save the ranges from the odometry readings
    laser_ranges = np.asarray(data.ranges)                                                      # float64

    #####################################################################################################
    # 3 rows for open3d Vector3dVector format
    # new_data = np.vstack([laser_ranges[0:120], laser_ranges[120:240], laser_ranges[240:360]])   # float64

    # Converting the LaserScan message into PointCloud2
    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(data)

    # Convert the PointCloud2 into a numpy array
    pc = ros_numpy.numpify(pc2_msg)
    height = pc2_msg.height
    width = pc2_msg.width
    np_points = np.zeros((height * width, 3), dtype=np.float64)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)

    # Format of the data is x and y, with z as zero
    # zeros_row = np.zeros(laser_ranges.size/2)
    # new_data = np.vstack([laser_ranges[0:180], laser_ranges[180:360], zeros_row])

    # zeros_row = np.zeros(laser_ranges.size)
    # new_data = np.vstack([laser_ranges, np.asarray(data.intensities), zeros_row])
    #####################################################################################################

    # Transpose the matrix for open3d format input
    # new_data = np.transpose(new_data)

    # Convert the numpy array to open3d type
    new_pcd.points = o3d.utility.Vector3dVector(np_points)

    # print(np.asarray(new_pcd.points))

    # Check that the pointclouds are different
    if old_pcd.points != new_pcd.points and np.asarray(old_pcd.points).size != 0:

        ############################################################
        # print('Average of old_pcd ', np.average(np.asarray(old_pcd.points)))
        # print('Average of new_pcd ', np.average(np.asarray(new_pcd.points)))
        # draw_registration_result(old_pcd, new_pcd, np.identity(4))
        ############################################################
        
        # Conduct ICP registration
        icp_registration(old_pcd, new_pcd)

    # Clear the new_data variable for the next iteration of points
    # new_data = np.array([])

    # # LATER IN Q1 CAN CHANGE THE MAX AND MIN RANGE BY WRITING TO RANGE_MIN AND RANGE_MAX


#################################################################################
# Subscriber for the trasnformation to calcualte the overall resultant transform 
# and get the x and y trajectory
def icp_transformation_callback(data):                                                          # data.data is a tuple

    global icp_transformation_matrix, result                                                    # result is float64

    # Add copy
    result_temp = copy.deepcopy(result)

    # Convert data into a numpy matrix
    icp_transformation_matrix = np.reshape(np.array(data.data), (4, 4))                         # float64

    # Multiply the two matrices
    # result = np.matmul(result_temp, icp_transformation_matrix)                                  # float64
    # result = np.matmul(icp_transformation_matrix, result_temp)
    result = np.matmul(result, np.linalg.inv(icp_transformation_matrix))

    # print("\n\nframes transformation:")
    # print(type(transformation_frames))
    # print("icp transformation:")
    # print(type(icp_transformation_matrix))
    # print("Product of the start with icp results:")
    # print(result)
    # print("\n\n")

    # Add the current x and y pose positions to the odom lists
    icpX.append(result[0, 3])                                                                   # float64
    icpY.append(result[1, 3])                                                                   # float64

#################################################################################
# Function sourced from Open3D Tutorials - visualises the alignment of the points
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

#################################################################################
# Function sourced from Open3D Tutorials - calculates the resultant transformation
# between point clouds using point to point ICP registration algorithm
# Source point cloud is the old, target point cloud is the new
def icp_registration(source, target):

    global reg_p2p

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    # Threshold for ICP correspondences
    threshold = 0.03

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    # print("Evaluation of initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source_temp, target_temp, threshold, init_trans)
    # print(evaluation)
    # draw_registration_result(source, target, init_trans)

    # print("\nTransformation from point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_temp, target_temp, threshold, init_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    # print("\nTransformation is:")
    # print(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)
    
    # Flatten the matrix for publishing
    icp_transformation = reg_p2p.transformation.flatten()                                       # float64

    flat_array = np.array(icp_transformation, dtype=np.float32)                                 # float32
    
    # Publish the ICP transformation as an array - needs to be float32
    icp_pub.publish(flat_array)
    

#################################################################################
def main():

    # Global vairables
    global odomX, odomY, new_pcd, old_pcd, icp_pub, icpX, icpY, result #, new_data

    # Global variable initialisation
    result = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]], dtype=np.float64)
    odomX = []
    odomY = []
    icpX = []
    icpY = []
    new_data = np.array([])

    # Open3d point clouds
    new_pcd = o3d.geometry.PointCloud()
    old_pcd = o3d.geometry.PointCloud()

    # Publisher for publishing the point cloud
    icp_pub = rospy.Publisher("ICP_transformation", Floats, queue_size=1)

    try:
        # Initialise a new node
        rospy.init_node('odom_ICP_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            print("loop start")

            # Subscribe to odometry, scan topics
            rospy.Subscriber("/odom", Odometry, odom_callback)#, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, laser_callback)#, queue_size=1)

            # Subscribe to the ICP transformation published data
            rospy.Subscriber("ICP_transformation", Floats, icp_transformation_callback)#, queue_size=1)
                
            # Sleep until next spin
            rate.sleep()

        # Plot the odometry trajectory 
        plt.plot(odomX, odomY, icpX, icpY)
        # plt.plot(icpX, icpY)
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
