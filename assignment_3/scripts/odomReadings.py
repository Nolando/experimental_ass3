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
    odomX.append(odom_pose.pose.position.x)
    odomY.append(odom_pose.pose.position.y)

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
    old_pcd = copy.deepcopy(new_pcd)

    # Save the ranges and intensities from the odometry readings - FIDDLE AROUND WITH FLOAT32 OR FLOAT64
    laser_ranges = np.array([data.ranges], np.float32)          # size: 1 x 360

    # If the variable is currently empty, will need to set new_data first
    if new_data.size < 360:
        new_data = laser_ranges

    # Check the data is not the same
    elif new_data is not laser_ranges:

        # Add 1D LIDAR data array to the next available row
        new_data = np.append(new_data, laser_ranges,axis=0)

    # Test if the variable has 3 columns - means n x 3 is full yet
    if new_data.size == 1080:

        # Transpose the matrix for open3d format input
        new_data = np.transpose(new_data)                       # size: 360 x 3

        # Convert the numpy array to open3d type
        new_pcd.points = o3d.utility.Vector3dVector(new_data)

        # Check that the pointclouds are different
        if old_pcd.points != new_pcd.points and np.asarray(old_pcd.points).size != 0:
            
            # Conduct ICP registration
            icp_registration(old_pcd, new_pcd)

        # Clear the new_data variable for the next iteration of points
        new_data = np.array([])

    # LATER IN Q1 CAN CHANGE THE MAX AND MIN RANGE BY WRITING TO RANGE_MIN AND RANGE_MAX

#################################################################################
# Subscriber for the trasnformation to calcualte the overall resultant transform 
# and get the x and y trajectory
def transformation_callback(data):

    # Convert data into a numpy matrix
    transformation_matrix = np.array(data)
    # transformation_matrix = np.reshape(np.asarray(data), (4, 4))

    # TO DO: SPLIT THIS INTO ELEMENTS AS IT IS ONLY SAYING THAT IT IS 1X1 VECTOR WITH ALL POINTS
    print("\n\n------------------")
    # print(np.asarray(data))
    transformation_matrix = np.array(list(map(np.float, transformation_matrix)))
    print(transformation_matrix)

#################################################################################
# Convert quaternion into a rotation matrix
def quaternion_to_rotation_matrix(Q):
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # Each element of the rot matrix using the conversion equation
    p00 = 2 * (q0 * q0 + q1 * q1) - 1   # First row
    p01 = 2 * (q1 * q2 - q0 * q3)
    p02 = 2 * (q1 * q3 + q0 * q2)
    p10 = 2 * (q1 * q2 + q0 * q3)       # Second row
    p11 = 2 * (q0 * q0 + q2 * q2) - 1
    p12 = 2 * (q2 * q3 - q0 * q1)
    p20 = 2 * (q1 * q3 - q0 * q2)       # Third row
    p21 = 2 * (q2 * q3 + q0 * q1)
    p22 = 2 * (q0 * q0 + q3 * q3) - 1

    # Rotation matrix
    rotation_matrix = np.array([[p00, p01, p02],
                                [p10, p11, p12],
                                [p20, p21, p22]])
    return rotation_matrix

#################################################################################
# Get transformation matrix from rotation in quaternions and translation vector
def get_transformation_matrix(quats, trans_matrix):
    
    # Change quaternion to rotation matrix
    rot_matrix = quaternion_to_rotation_matrix(quats)   # type: numpy.ndarray

    # Convert to numpy array
    trans_matrix = np.array(trans_matrix)       # type: numpy.ndarray

    # Create the transformation matrix
    tf_matrix = np.array([[rot_matrix[0,0], rot_matrix[0,1], rot_matrix[0,2], trans_matrix[0]],
                         [rot_matrix[1,0], rot_matrix[1,1], rot_matrix[1,2], trans_matrix[1]],
                         [rot_matrix[2,0], rot_matrix[2,1], rot_matrix[2,2], trans_matrix[2]],
                         [0, 0, 0, 1]])
    
    # Return the tf
    return tf_matrix

#################################################################################
# Function sourced from Open3D Tutorials - visualises the alignment of the points
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # o3d.visualization.draw_geometries([source_temp, target_temp])

#################################################################################
# Function sourced from Open3D Tutorials - calculates the resultant transformation
# between point clouds using point to point ICP registration algorithm
# Source point cloud is the old, target point cloud is the new
def icp_registration(source, target):

    global reg_p2p
        
    # Threshold for ICP correspondences
    threshold = 0.02

    # Initial transformation is estimated as the identiy matrix
    init_trans = np.identity(4)

    # Calculate transformation
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, init_trans,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=10000))
    icp_transformation = reg_p2p.transformation.flatten()
    
    # draw_registration_result(source, target, reg_p2p.transformation)

    # Publish the ICP transformation as an array
    flat_array = np.array(icp_transformation, dtype=np.float32)
    icp_pub.publish(flat_array)
    

#################################################################################
def main():

    # Global vairables
    global odomX, odomY, new_data, new_pcd, old_pcd, icp_pub, transformation_matrix
    odomX = []
    odomY = []
    new_data = np.array([])

    # Open3d point clouds
    new_pcd = o3d.geometry.PointCloud()
    old_pcd = o3d.geometry.PointCloud()

    # Publisher for publishing the point cloud
    icp_pub = rospy.Publisher("ICP_transformation", numpy_msg(Floats), queue_size=1)

    try:
        # Initialise a new node
        rospy.init_node('odom_ICP_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Listener variable for tf topic
        listener = tf.TransformListener()

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            # Subscribe to odometry, scan topics
            rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)

            # Subscribe to the translation (linear transformation in x, y, z) 
            # and rotation (quaternion in x, y, z, w) of the child relative to parent frame
            try:
                (tf_trans, tf_rot) = listener.lookupTransform('/base_scan', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Get the transformation from /odom (world) to the LIDAR scanner frame
            transformation = get_transformation_matrix(tf_rot, tf_trans)    # type: numpy.ndarray

            # Subscribe to the ICP transformation published data
            rospy.Subscriber("ICP_transformation", Floats, transformation_callback, queue_size=1)
                
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
