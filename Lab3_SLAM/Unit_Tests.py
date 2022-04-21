import numpy as np

from Robot import Robot
from Landmark import Landmark
import matplotlib.pyplot as plt
import geometry

import random

from LandmarkSLAM_students import LeastSquaresSolver

# Covariance to experiment with
pose_measurement_covariance = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])*10**(-2)
landmark_measurement_covariance = np.array([[1.0,0.0],[0.0, 1.0]])*10**(-4)


def plotState(robot_states, landmark_states, unit_number,flag):
    plt.figure(1)
    plt.clf()
    num_states = len(robot_states)
    num_landmarks = len(landmark_states)

    traj_x = []
    traj_y = []

    odo_x = []
    odo_y = []



    for i in range(num_states):
        traj_x.append(robot_states[i].get_current()[0])
        traj_y.append(robot_states[i].get_current()[1])
        odo_x.append(robot_states[i].get_initial()[0])
        odo_y.append(robot_states[i].get_initial()[1])
    
    landmark_x = []
    landmark_y = []
    landmark_px = []
    landmark_py = []
    for i in range(num_landmarks):
        landmark_x.append(landmark_states[i].get_current()[0])
        landmark_y.append(landmark_states[i].get_current()[1])
        landmark_px.append(landmark_states[i].get_initial()[0])
        landmark_py.append(landmark_states[i].get_initial()[1])

    plt.plot(traj_x, traj_y, 'bo--')
    plt.plot(odo_x, odo_y, 'go--')
    plt.plot(landmark_x, landmark_y, 'rx')
    plt.plot(landmark_px, landmark_py, 'cx')
    plt.grid(True)
    if flag:
        grade = "FAILED"
    else:
        grade = "PASSED"
    plt.title("Unit Test #{}: {}".format(unit_number,grade))
    plt.xlabel("$x$ (m)")
    plt.ylabel("$y$ (m)")
    plt.legend(["Solved Trajectory","Odometry","Solved Landmark","Measured Landmark"])
    plt.savefig("output_test_{}.png".format(unit_number))


def unit_test():

    print("*****************************************")
    print("Preparing to execute unit tests...")
    print("*****************************************")
    print("Unit Test #1: Linear Robot Motion.")

    robot_poses = np.array([[0,0,0],
                            [1,0,0],
                            [2,0,0],
                            [3,0,0],
                            [4,0,0]]).T
    
    landmark_gt = np.array([[1,1],
                            [2,1]]).T

    run_unit_test(robot_poses,landmark_gt,1)

    


    print("*****************************************")
    print("Unit Test #2: Once more, with rotation!")


    robot_poses = np.array([[0,0,0],
                            [1,0,np.pi/4],
                            [2,0,np.pi/2]]).T
    
    landmark_gt = np.array([[1,1],
                            [2,1]]).T



    run_unit_test(robot_poses,landmark_gt,2)

    
    print("*****************************************")
    print("Unit Test #3: Now we're SLAM-min'!")


    robot_poses = np.array([[0,0,0],
                            [1,1,np.pi/4],
                            [2,2,np.pi/2],
                            [3,2,np.pi/2],
                            [3,3,np.pi/2],
                            [2,4,np.pi/2]]).T
    
    landmark_gt = np.array([[1,2],
                            [2,3]]).T

    run_unit_test(robot_poses,landmark_gt,3)

    


def eval_unit_test(soln_robot, robot_poses, soln_landmark, landmark_gt,unit_number):

    fail_flag = False
    i = 0
    for robot in soln_robot:
        print("Position_ID: " + str(robot.get_index()))
        print("GROUND TRUTH: " + str(robot_poses[:,i]))
        print( "SOLVED: " + str(robot.get_current().T))
        error = robot_poses[:,i].reshape(3,1)-robot.get_current().reshape(3,1)
        print("ERROR: " + str(error.T))
        if np.linalg.norm(error) > 1e-1:
            print("FAILED")
            fail_flag = True
        else:
            print("OK")

        i += 1
    
    i = 0
    for landmark in soln_landmark:
        print("Landmark_ID: " + str(landmark.get_index()))
        print("GROUND TRUTH: " + str(landmark_gt[:,i]))
        print("SOLVED: " + str(landmark.get_current().T))
        error = landmark_gt[:,i].reshape(2,1)-landmark.get_current().reshape(2,1)
        print("ERROR: "+str(error.T))
        if np.linalg.norm(error) > 1e-1:
            print("FAILED")
            fail_flag = True
        else:
            print("OK")
        i+= 1

    print("\n===\nUNIT TEST #{}:\n".format(unit_number))
    if fail_flag:
        print("FAILED.\n===\n")
    else:
        print("PASSED.\n===\n")
    
    return fail_flag

def construct_unit_test(robot_poses,landmark_gt, meas_error = 0.005, odo_error = 0.5):

    robot_states = []
    for i in range(robot_poses.shape[1]):

        meas_indices = []
        measurements = []
        
        for j in range(landmark_gt.shape[1]):
            meas_indices += [j]
            rel_measurement, _, _ = geometry.Absolute2RelativeXY(robot_poses[:,i],landmark_gt[:,j])
            measurements += [rel_measurement.reshape(2,1)+np.array([np.random.standard_normal()*meas_error,
                            np.random.standard_normal()*meas_error]).reshape(2,1)]

        if i == 0:
            
            robot_states += [Robot(robot_poses[:,i].T+ np.array([np.random.standard_normal()*odo_error,0,0.0]), 
                            np.array([np.random.standard_normal()*odo_error,0,0.0]).T + np.array([np.random.standard_normal()*odo_error,0,0.0]),
                            i,measurements,meas_indices, pose_measurement_covariance)]
        else:
            motion_error = (robot_poses[:,i]-robot_poses[:,i-1]) + np.array([np.random.standard_normal()*odo_error,0,0.0])
            robot_states += [Robot(robot_states[-1].get_current()+ motion_error, motion_error,i,measurements,meas_indices, pose_measurement_covariance)]
    
    landmark_states = []

    for i in range(landmark_gt.shape[1]):

        landmark_states += [Landmark(landmark_gt[:,i]+np.array([np.random.standard_normal()*meas_error*5,
                            np.random.standard_normal()*meas_error*5]),i, landmark_measurement_covariance)]

    solver = LeastSquaresSolver()
    
    return solver.solve(robot_states,landmark_states)


def run_unit_test(robot_poses,landmark_gt,number):
    soln_robot, soln_landmark = construct_unit_test(robot_poses,landmark_gt)

    flag = eval_unit_test(soln_robot,robot_poses,soln_landmark,landmark_gt,number)

    plotState(soln_robot, soln_landmark,number, flag)

if __name__ == "__main__":
    random.seed(2206)
    unit_test()



