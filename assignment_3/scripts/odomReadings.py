#!/usr/bin/env python
# Machine Unlearning

# Subscribe to the odom topic to get the turtlebot odometry readings  
import rospy
import copy
import numpy as np
import open3d as o3d
import tf                                       # /tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry               # /odom
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan           # /scan
from rospy.numpy_msg import numpy_msg
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
    new_data = np.vstack([laser_ranges[0:120], laser_ranges[120:240], laser_ranges[240:360]])   # float64

    # Transpose the matrix for open3d format input
    new_data = np.transpose(new_data)

    # Convert the numpy array to open3d type
    new_pcd.points = o3d.utility.Vector3dVector(new_data)

    # Check that the pointclouds are different
    if old_pcd.points != new_pcd.points and np.asarray(old_pcd.points).size != 0:

        ############################################################
        draw_registration_result(old_pcd, new_pcd, np.identity(4))
        ############################################################
        
        # Conduct ICP registration
        icp_registration(old_pcd, new_pcd)

    # Clear the new_data variable for the next iteration of points
    new_data = np.array([])

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
    result = np.matmul(icp_transformation_matrix, result_temp)                                  # float64

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
    threshold = 0.2

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    print(source)
    print(target)

    # print("Evaluation of initial alignment")
    evaluation = o3d.registration.evaluate_registration(
        source_temp, target_temp, threshold, init_trans)
    # print(evaluation)
    # draw_registration_result(source, target, init_trans)

    # print("\nTransformation from point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
        source_temp, target_temp, threshold, init_trans,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=1000))
    # print(reg_p2p)
    # print("\nTransformation is:")
    # print(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)

    # Calculate transformation
    # reg_p2p = o3d.registration.registration_icp(
    #     source, target, threshold, init_trans,
    #     o3d.registration.TransformationEstimationPointToPoint(),
    #     o3d.registration.ICPConvergenceCriteria(max_iteration=10000))

    # draw_registration_result(source, target, init_trans)
    
    # Flatten the matrix for publishing
    icp_transformation = reg_p2p.transformation.flatten()                                       # float64

    flat_array = np.array(icp_transformation, dtype=np.float32)                                 # float32
    
    # Publish the ICP transformation as an array - needs to be float32
    icp_pub.publish(flat_array)
    

#################################################################################
def main():

    # Global vairables
    global odomX, odomY, new_data, new_pcd, old_pcd, icp_pub, icpX, icpY, result

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

        # Listener variable for tf topic
        listener = tf.TransformListener()

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            print("loop start")

            # Subscribe to odometry, scan topics
            rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)

            # Subscribe to the translation (linear transformation in x, y, z) 
            # and rotation (quaternion in x, y, z, w) of the child relative to parent frame
            try:
                # (tf_trans, tf_rot) = listener.lookupTransform('/base_scan', '/odom', rospy.Time(0))     # Think this is more right
                (tf_trans, tf_rot) = listener.lookupTransform('/odom', '/base_scan', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Subscribe to the ICP transformation published data
            rospy.Subscriber("ICP_transformation", Floats, icp_transformation_callback, queue_size=1)
                
            # Sleep until next spin
            rate.sleep()

        # Plot the odometry trajectory 
        plt.plot(odomX, odomY, icpX, icpY)
        # plt.plot(icpX, icpY)
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
