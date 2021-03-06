#! /usr/bin/env python
#
# Author: Tejaswi Digumarti (tejaswi.digumarti@sydney.edu.au)
# Description: This code demonstrates the control of the robot arm and its gripper over ROS using python.

import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface


def demo_print_information(mgpi):
    """
    Prints some useful information.
    :param mgpi: An object of the MoveGroupPythonInterface class
    """
    print("\n**********************************")
    print("  2. Controlling in Joint Space")
    print("**********************************")
    print("  2.1 Print Basic Information")
    print("----------------------------------")
    raw_input("Press Enter to continue...")

    mgpi.print_useful_info()  # Print useful information

    print("\n  2.2 Current Robot State")
    print("---------------------------------")
    raw_input("Press Enter to continue...")

    mgpi.print_robot_state()  # Print the current state of the robot


def demo_joint_space_control(mgpi):
    """
    Demonstrates the use of joint angles to control the arm.
    :param mgpi: An object of the MoveGroupPythonInterface class
    """
    print("\n**********************************")
    print("  3. Controlling in Joint Space")
    print("**********************************")
    print("3.1 Move the robot to predefined \"home\" state.")
    print("---------------------------------------------------")
    print("- This home state is defined in universal_robot/ur5_e_moveit_config/config/ur5e.srdf")
    print("- Please DO NOT edit this file.")
    raw_input("Press Enter to continue...")

    # predefined state "home" is defined in universal_robot/ur5_e_moveit_config/config/ur5e.srdf
    mgpi.move_to_joint_state(mgpi.home_joint_angles)

    print("\n3.2 Move the robot to a desired joint state.")
    print("-----------------------------------------------")
    print("- The desired joint state in degrees is [-45, -30, 60, -135, 75, 20]")
    print("- The angles are to be entered in radians.")
    print("- The order of the angles is from the base joint to the wrist joint.")
    raw_input("Press Enter to continue..")

    mgpi.move_to_joint_state([-0.78, -0.7, 1.05, -2.36, 1.39, 0.35])  # values are in radians

    print("\nPress Enter to return back to home state.")
    raw_input()
    mgpi.move_to_joint_state(mgpi.home_joint_angles)


def demo_cartesian_space_control(mgpi):
    """
    Demonstrates the use of pose in Cartesian space to control the arm.
    :param mgpi: An object of the MoveGroupPythonInterface class
    """
    print("\n*************************************************")
    print("  4. Moving the end-effector in Cartesian space")
    print("*************************************************")
    print("- The desired pose is expressed using the geometry_msgs.msg.Pose() message.")
    print("- Position is a point in 3D space (x, y, z)")
    print("- Orientation is a quaternion")
    raw_input("Press Enter to continue..")

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.15
    target_pose.position.y = 0.45
    target_pose.position.z = 0.6
    target_pose.orientation.x = 1.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0
    mgpi.move_eef_to_pose(target_pose)

    raw_input("\nPress Enter to return back to home state.")

    mgpi.move_to_joint_state(mgpi.home_joint_angles)


def callback(input_msg):
    print(input_msg)


def main():
    try:
        rospy.loginfo("Setting up simulation environment...")
        rospy.loginfo("------------------------------------")
        mgpi = MoveGroupPythonInteface()  # Create and Initialize the interface
        rospy.loginfo("Done")

        # Write any code you want here - such as setting pre-determined waypoints.
        # 1. You may use the functions provided to move the robot to specific poses
        #    or use the GUI of moveit_calibration
        #
        # 2. You can write as many additional functions as you need.
        #    But please keep all your code in this file.
        #
        # 3. Necessary modules have been imported.
        #    You may only import additional system or standard python modules.
        #    If importing any special modules, please check with your tutors if you are allowed to do so.
        #    Importing 'tf' and 'tf2' packages for quaternion/rotation transformations is allowed.
        #
        # 4. Please write clean code and comment is sufficiently.
        #
        # 5. Be aware that the path planner in MoveIt! is inherently non-deterministic.
        #    Paths between different runs may be completely different.

    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return


if __name__ == '__main__':
    main()
