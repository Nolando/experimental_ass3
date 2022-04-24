#! /usr/bin/env python
#
# Group: Machine Unlearning
# Description: 

import rospy
import geometry_msgs.msg
import copy
import os
import sys
import numpy as np
import open3d as o3d
from move_group_interface import MoveGroupPythonInteface


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
def icp_registration(source, target, threshold, init_trans):

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
        # Make sure current directory is ur5espace, read source and target clouds
        source = o3d.io.read_point_cloud("src/assignment_2/pos1_pointclouds/806.388000000.pcd")
        # target = o3d.io.read_point_cloud("src/assignment_2/pos2_pointclouds/954.201000000.pcd")
        target = o3d.io.read_point_cloud("src/assignment_2/pos3_pointclouds/944.174000000.pcd")

        # Threshold for ICP correspondences
        threshold = 0.02

        # Initial transformation is estimated as the identiy matrix
        init_trans = np.identity(4)
        print("\Initial transformation estimate is identiy matrix")
        print(init_trans)

        # Wait for user enter to visualise the points with initial estimate
        print("\nPress Enter to visualise points")
        raw_input()
        draw_registration_result(source, target, init_trans)

        # Get the transformation between clouds using ICP
        icp_registration(source, target, threshold, init_trans)

    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return


if __name__ == '__main__':
    main()
