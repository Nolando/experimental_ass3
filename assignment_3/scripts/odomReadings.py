#!/usr/bin/env python3
# Machine Unlearning

# Note - things to write about in report
# - chacking the different types of all variables
# - different methods of inputing data into open3d format
#       - trying using the single data array open3d utility functions
#       - having three rows of same data like it is a col of data
#       - spliting the points in 2 and in 3 to input into open3d
#       - only using the x and y values (since 2D)
#       - ended up going with converting to pointcloud2 then to numpy array (open3d)
# - subscriber diagrams, rosnodes
# - data flow diagram

# Subscribe to the odom topic to get the turtlebot odometry readings  
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
# Converts the angles and accounts for cosine and sine angles. Obtained from 
# Assignment 3 provided code.
def pi2pi(angle):
    dp = 2*np.pi
    if angle <= -dp or angle >= dp:
        angle = angle % dp
    if angle >= np.pi:
        angle = angle - dp
    if angle <= -np.pi:
        angle = angle + dp
    return angle

#################################################################################
# Subscriber callback function for odom saves data
def odom_callback(data):

    # Global variables for odom data
    global odom_pose, odomNextX, odomNextY

    # Save the data from the odometry readings
    odom_pose = data.pose

    # Next odom x and y values
    odomNextX = np.float64(odom_pose.pose.position.x)
    odomNextY = np.float64(odom_pose.pose.position.y)

    # Add the current x and y pose positions to the odom lists
    odomX.append(odomNextX)                                         # float64
    odomY.append(odomNextY)                                         # float64

####################### Relative and Absolute Error #############################
# Function to calculate the relative and absolute trajectory error 
def error(lidarDataX, lidarDataY, odomDataX, odomDataY):

    print("Calculating errors: ")

    # Error lists for average calculation
    relativeErrorsX = []
    absoluteErrorsX = []
    relativeErrorsY = []
    absoluteErrorsY = []

    # Set the data length to the length of the smaller data set
    if len(odomDataX) > len(lidarDataX):
        dataLength = len(lidarDataX)
    else:
        dataLength = len(odomDataX)

    # Check if it is time to output the errors for the next interval
    for index in range(0, dataLength):

        # Calculate the relative and absolute error for the current pair of points
        currentRelativeX = abs(odomDataX[index] - lidarDataX[index])
        currentRelativeY = abs(odomDataY[index] - lidarDataY[index])
        currentAbsoluteX = currentRelativeX/odomDataX[index]
        currentAbsoluteY = currentRelativeY/odomDataY[index]

        # Add the current errors to their respective lists
        relativeErrorsX.append(currentRelativeX)
        relativeErrorsY.append(currentRelativeY)
        absoluteErrorsX.append(currentAbsoluteX)
        absoluteErrorsY.append(currentAbsoluteY)

    # Error sum variables for a single interval
    totalRelativeX = 0
    totalAbsoluteX = 0
    totalRelativeY = 0
    totalAbsoluteY = 0

    # Calculate the sum of the relative and absolute errors
    for errorIndex in range(0, dataLength):
        totalRelativeX += relativeErrorsX[errorIndex]
        totalRelativeY += relativeErrorsY[errorIndex]
        totalAbsoluteX += absoluteErrorsX[errorIndex]
        totalAbsoluteY += absoluteErrorsY[errorIndex]

    # Calculate average errors and display
    averageRelativeX = totalRelativeX/len(relativeErrorsX)
    averageRelativeY = totalRelativeY/len(relativeErrorsY)
    averageAbsoluteX = totalAbsoluteX/len(absoluteErrorsX)
    averageAbsoluteY = totalAbsoluteY/len(absoluteErrorsY)

    print("Average Relative Error: ", averageRelativeX, averageRelativeY)
    print("Average Absolute Error: ", averageAbsoluteX, averageAbsoluteY)

#################################################################################
# Subscriber callback function for the laser scan saves data
def laser_callback(data):

    global new_data, laser_ranges

    # Update old pointcloud variable - NEED TO COPY OR ELSE IS PYTHON REFERENCES
    old_pcd = copy.deepcopy(new_pcd)

    data_copy = copy.deepcopy(data)

    # Save the ranges from the odometry readings
    laser_ranges = np.asarray(data.ranges)                                                      # float64
    laser_copy = copy.deepcopy(laser_ranges)

    # Intensities for the odom readings
    laser_intense = np.asarray(data.intensities)  
    intense_copy = copy.deepcopy(laser_intense)

    # Discard the values outside of the max and min ranges - can use this later for range filter
    data_copy.ranges = tuple(laser_copy[(laser_copy > data.range_min) & (laser_copy < data.range_max)])
    data_copy.intensities = tuple(intense_copy[(laser_copy > data.range_min) & (laser_copy < data.range_max)])

    #####################################################################################################
    # 3 rows for open3d Vector3dVector format
    # new_data = np.vstack([laser_ranges[0:120], laser_ranges[120:240], laser_ranges[240:360]])   # float64

    # Format of the data is x and y, with z as zero
    # zeros_row = np.zeros(laser_copy.size/2)
    # new_data = np.vstack([laser_copy[0:zeros_row.size], laser_copy[zeros_row.size:2*zeros_row.size], zeros_row])

    # Converting the LaserScan message into PointCloud2
    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(data_copy)

    # Convert the PointCloud2 into a numpy array
    pc = ros_numpy.numpify(pc2_msg)
    height = pc2_msg.height
    width = pc2_msg.width
    np_points = np.zeros((height * width, 3), dtype=np.float64)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    #####################################################################################################

    # Convert the numpy array to open3d type
    new_pcd.points = o3d.utility.Vector3dVector(np_points)

    # Check that the pointclouds are different
    if old_pcd.points != new_pcd.points and np.asarray(old_pcd.points).size != 0:

        ############################################################
        # draw_registration_result(old_pcd, new_pcd, np.identity(4))
        ############################################################
        
        # Conduct ICP registration
        icp_registration(old_pcd, new_pcd)

    # # LATER IN Q1 CAN CHANGE THE MAX AND MIN RANGE BY WRITING TO RANGE_MIN AND RANGE_MAX


#################################################################################
# Subscriber for the trasnformation to calcualte the overall resultant transform 
# and get the x and y trajectory
def icp_transformation_callback(data):                                                          # data.data is a tuple

    global icp_transformation_matrix, result, icpNextX, icpNextY                                # result is float64

    # Add copy
    result_temp = copy.deepcopy(result)

    # Convert data into a numpy matrix
    icp_transformation_matrix = np.reshape(np.array(data.data), (4, 4))                         # float64

    # Multiply the two matrices
    result = np.matmul(result_temp, icp_transformation_matrix)

    ####################################################
    # WILL NEED TO ACCOUNT FOR THE SINE AND COSINE CHANGES - LOOK AT Q2 CODE - NEED AN IF STATEMENT TO ACCOUNT FOR PI SIGN CHANGES
    # In geometry.py in LAB3_SLAM
    ####################################################

    # Next x and y values for icp
    icpNextX = result[0, 3]
    icpNextY = result[1, 3]

    # Add the current x and y pose positions to the odom lists
    icpX.append(icpNextX)                                                                   # float64
    icpY.append(icpNextY)     
    

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

    global reg_p2p

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    # Threshold for ICP correspondences
    threshold = 0.03

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    # Conduct point to point ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_temp, target_temp, threshold, init_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=3000))

    ############################################################
    # draw_registration_result(source_temp, target_temp, reg_p2p.transformation)
    ############################################################
    print(reg_p2p.transformation)
    
    # Flatten the matrix for publishing
    icp_transformation = reg_p2p.transformation.flatten()                                       # float64
    flat_array = np.array(icp_transformation, dtype=np.float32)                                 # float32
    
    # Publish the ICP transformation as an array - needs to be float32
    icp_pub.publish(flat_array)

#################################################################################
# def realTimeErrors(icpNextX, icpNextY, odomNextX, odomNextY):
    
#     # Limit value for the current iteration
#     iterationLimit = 10

#     # Calculate the next errors
#     nextRelativeX = abs(icpNextX - odomNextX)                                                              # float64
#     nextRelativeY = abs(icpNextY - odomNextY)                                                              # float64
#     nextAbsoluteX = nextRelativeX/odomNextX
#     nextAbsoluteY = nextRelativeY/odomNextY

#     # Add to the error sums
#     realRx += nextRelativeX
#     realRy += nextRelativeY
#     realAx += nextAbsoluteX
#     realAy += nextAbsoluteY
#     print("Hello")
#     # Check if the new average error is ready 
#     if iterations > iterationLimit:
        
#         # Display the new average errors
#         #print(realRx/10)
#         print("DONE")
#         # Reset the error lists
#         realRx = 0
#         realRy = 0
#         realAx = 0
#         realAy = 0

#     # Update iterations
#     iterations += 1


#################################################################################
def main():

    # Global vairables
    global odomX, odomY, new_pcd, old_pcd, icp_pub, icpX, icpY, result, iterations
    #global realRx, realRy, realAx, realAy

    #realRx = 0
    #realRy = 0
    #realAx = 0
    #realAy = 0

    #iterations = 0

    # Global variable initialisation
    result = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]], dtype=np.float64)         # Could change this to /odom frame coord?
    odomX = []
    odomY = []
    icpX = []
    icpY = []

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

            # Subscribe to odometry, scan topics
            rospy.Subscriber("/odom", Odometry, odom_callback)
            rospy.Subscriber("/scan", LaserScan, laser_callback)

            # Subscribe to the ICP transformation published data
            rospy.Subscriber("ICP_transformation", Floats, icp_transformation_callback)
                
            # Error calc and print function
            #realTimeErrors(icpNextX, icpNextY, odomNextX, odomNextY)
            
            # Sleep until next spin
            rate.sleep()

        error(icpX, icpY, odomX, odomY)

        # Plot the odometry trajectory 
        plt.plot(odomX, odomY, icpX, icpY)
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
