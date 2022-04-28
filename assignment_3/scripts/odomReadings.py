#!/usr/bin/env python

# Subscribe to the odom topic to get the turtlebot odometry readings  
from pickle import FALSE
from tkinter import TRUE
import rospy
import copy
import numpy as np
import open3d as o3d
import laser_geometry as lg
import tf                                       # /tf
from nav_msgs.msg import Odometry               # /odom
from sensor_msgs.msg import LaserScan, PointCloud2           # /scan
from tf.msg import tfMessage
from scipy.spatial.transform import Rotation

#################################################################################
# Global variables
lp  = lg.LaserProjection()      # Laser Projection 

# Publisher for publishing the point cloud
pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    
#################################################################################
# Subscriber callback function for odom saves data
def odom_callback(data):

    # Global variables for odom data
    global odom_pose, odom_twist

    # Save the data from the odometry readings
    odom_pose = data.pose
    odom_twist = data.twist

    # Test that the data is correctly being subscribed to
    # print(odom_twist)

#################################################################################
# Subscriber callback function for the laser scan saves data
def laser_callback(data):
    print("laser callback (start)")

    # Global variables for odom data
    global laser_ranges, laser_intensities, pc2_new

    # Save the ranges and intensities from the odometry readings
    laser_ranges = data.ranges
    laser_intensities = data.intensities

    # Convert LaserScan message to PointCloud2
    pc2_new = lp.projectLaser(data)

    # Publish the pointcloud
    pc_pub.publish(pc2_new)
    print("laser callback (end)")
    # LATER IN Q1 CAN CHANGE THE MAX AND MIN RANGE BY WRITING TO RANGE_MIN AND RANGE_MAX

# Callback to make the new pointcloud the old one
def pointcloud_callback(data):
    global pc2_old
    pc2_old = data
    print("pointcloud callback")

#################################################################################
# Subscriber callback function for tf saves data
def tf_callback(data):

    # Global variables for odom data
    global tf_transform

    # Save the data from the odometry readings
    tf_transform = data.transforms

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
    o3d.visualization.draw_geometries([source_temp, target_temp])

#################################################################################
# Function sourced from Open3D Tutorials - calculates the resultant transformation
# between point clouds using point to point ICP registration algorithm
def icp_registration():

    # Get the source and target pointclouds from the LIDAR sensor - CHANGE TO GET SOURCE AT ONE 
    # INSTANCE AND TARGET LIKE 0.5 SECONDS LATER - SO CAN COMPARE TRANSFORMATION BETWEEN ROBOT MOVEMENT
    source = pc2_old
    target = pc2_new

    # CURRENTLY THE SOURCE AND TARGET ARE SAME

    print("print target (pc2_new) in icp reg fn")
    raw_input()
    print(target)

    print("print source (pc2_old) in icp reg fn")
    raw_input()
    print(source)


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
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, init_trans,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=20000))
    print(reg_p2p)
    print("\nTransformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)

#################################################################################
def main():
    try:
        # Initialise a new node
        rospy.init_node('odom_ICP_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Can't estimate with ICP on first loop iteration
        ICP_bool = False

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            print("at loop start")
            raw_input()

            # Subscribe to odometry, scan and tf topics
            # rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)
            # rospy.Subscriber("/tf", tfMessage, tf_callback, queue_size=1)

            # Calls the ICP function only if two point clouds obtained
            if ICP_bool:
                icp_registration()

            # New point cloud becomes the old
            rospy.Subscriber("converted_pc", PointCloud2, pointcloud_callback, queue_size=1)
            
            # Will have two point clouds from first iteration onwards
            ICP_bool = True

            # Sleep until next spin
            rate.sleep()


    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return
   
if __name__ == '__main__':
    main()
