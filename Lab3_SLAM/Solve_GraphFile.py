from matplotlib.pyplot import pause
from LandmarkSLAM_students import LeastSquaresSolver, SLAM
import geometry
from Robot import Robot
from Landmark import Landmark
import rospy

from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
import numpy as np
import matplotlib.pyplot as plt


file = 'graph.txt'


class Solve_File(SLAM):
    def __init__(self, file):
        
        self.state_dim = 3
        self.landmark_dim = 2
        self.seen_landmarks_labels = []

        self.pub_traj = rospy.Publisher('/robot_trajectory', Path, queue_size=5)
        self.pub_landmarks = rospy.Publisher('/landmarks_state', MarkerArray, queue_size=5)

        self.solver = LeastSquaresSolver()

        self.robot_poses, self.landmarks = self.load_file(file)
        self.robot_poses, self.landmarks = self.solver.solve(self.robot_poses, self.landmarks)
        self.publish_trajectory_history()
        self.publish_landmark_state()
        plotState(self.robot_poses, self.landmarks)


    def load_file(self, file):
        
        robot_poses = []
        landmarks = []
        position = np.zeros((3,1))

        f = open(file, 'r')
        for line in f:
            info = line.split(' ')
            if info[0] == 'ODOM3D':
                index = int(info[1])
                odo = np.array([info[2], info[4], info[6]], dtype=np.float)
                pose_covariance = np.array([[info[3], 0.0, 0.0],
                                                        [0.0, info[5], 0.0],
                                                        [0.0, 0.0, info[7]]], dtype=np.float)

                position, _, _ = geometry.Relative2AbsolutePose(position,odo)
                robot_poses.append(Robot(position.reshape((3,1)),odo.reshape(3,1),index,[],
                                        [], pose_covariance))
            elif info[0] == 'MEAS2D':
                landmark_index = int(info[2])
                meas = np.array([info[3],info[5]], dtype=np.float).reshape(2,1)
                meas_covariance = np.array([[info[4], 0.0],
                                    [0.0, info[6]]], dtype=np.float)
                robot_poses[-1].set_new_measurement(meas)
                robot_poses[-1].set_new_measurement_index(landmark_index)
                
                if landmark_index not in self.seen_landmarks_labels:
                    self.seen_landmarks_labels.append(landmark_index)
                    #print(landmark_index)
                    meas_location, _, _ = geometry.Relative2AbsoluteXY(position, meas)
                    landmarks.append(Landmark(meas_location, landmark_index, meas_covariance))

        return robot_poses, landmarks


def plotState(robot_states, landmark_states):

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

    plt.plot(traj_x, traj_y, 'b--')
    plt.plot(odo_x, odo_y, 'g--')
    plt.plot(landmark_x, landmark_y, 'rx')
    plt.plot(landmark_px, landmark_py, 'cx')
    plt.grid(True)

    plt.title("Solved SLAM")
    plt.xlabel("$x$ (m)")
    plt.ylabel("$y$ (m)")
    plt.legend(["Solved Trajectory","Odometry","Solved Landmark","Measured Landmark"])
    plt.savefig("solved_graph.png")


if __name__ == "__main__":
    rospy.init_node('graph_solver')
    Solve_File(file)