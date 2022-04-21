 #!/usr/bin/env python

import sys
sys.path.append("/home/dtejaswi/Documents/Teaching/mtrx5700/2021/assignments/ass3_space/devel/lib/python2.7/dist-packages")

import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from landmarks_msg.msg import Landmark_Msg, Landmarks_Msg
from tf_conversions import transformations
from geometry import Absolute2RelativeXY
from geometry import pi2pi
from visualization_msgs.msg import Marker, MarkerArray

# These values represent the standard deviation in the position of a landmark
# as a percentage of the distance to the landmark.
# Change these values and analyse the effect it has on the Kalman filter update.
std_dev_landmark_x = 0.01   # std dev is 2% of the distance, or 1cm per metre
std_dev_landmark_y = 0.01   # std dev is 2% of the distance, or 1cm per metre


class PublishLandmarks:

    def __init__(self):
        self.pub_landmarks = rospy.Publisher('/landmarks', Landmarks_Msg, queue_size=5)
        self.pub_landmark_markers = rospy.Publisher('/landmark_markers_gt', MarkerArray, queue_size=5)
        self.gazebo_srv = None

    def set_gazebo_service(self, gazebo_srv):
        self.gazebo_srv = gazebo_srv

    def create_landmark_marker(self, id, x, y):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.id = id
        marker.type = 3
        marker.action = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.
        marker.pose.orientation.x = 0.
        marker.pose.orientation.y = 0.
        marker.pose.orientation.z = 0.

        # Purple: 0, Orange: 1, Yellow: 2, Blue: 3,
        # Green: 4, Red: 5, Black: 6, Turquoise: 7
        if id == 0:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        elif id==1:
            marker.color.r = 1.0
            marker.color.g = 0.64
            marker.color.b = 0.0
        elif id==2:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif id==3:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        elif id==4:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif id==5:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif id==6:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif id==7:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.color.a = 0.7
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.frame_locked = False             # keep this False
        marker.lifetime = rospy.Duration(0.1)  # lifetime of a marker
        return marker

    def callback(self, data):
        # Simulation: Read the position of the landmarks from gazebo in 
        # the absolute frame and transform them relative to the robot's frame
        # Check for the landmarks within sensor's the field of view.
        # Simulation labels: Purple: 0, Orange: 1, Yellow: 2, Blue: 3, Green: 4, Red: 5, Black: 6, Turquoise: 7

        msg_landmarks = Landmarks_Msg()

        #robot pose -> Ground truth pose from gazebo
        robot_pose_gt = data.pose[10]
        yaw = transformations.euler_from_quaternion([robot_pose_gt.orientation.x,
                                                     robot_pose_gt.orientation.y,
                                                     robot_pose_gt.orientation.z,
                                                     robot_pose_gt.orientation.w])[2]

        robot_pose_abs_gt = [[robot_pose_gt.position.x], [robot_pose_gt.position.y], [pi2pi(yaw)]]
        marker_array_msg = MarkerArray()

        # In the gazebo/model_states, the cylinders have an index in the range [2,9]
        for i in range(0, 8):
            landmark_position_abs_gt = [data.pose[i+2].position.x,
                                        data.pose[i+2].position.y]
            # convert from absolute to relative coordinate frame using the ground
            # truth pose of the robot
            [landmark_position_rel_gt, _, _] = Absolute2RelativeXY(robot_pose_abs_gt,
                                                                   landmark_position_abs_gt)           

            # print("landmark {} at relative location: {}, {}".format(i-2, landmark_position_rel_gt[0][0], landmark_position_rel_gt[1][0]))

            # check for infront and within field of viewlandmark_position_rel_gt
            if landmark_position_rel_gt[0][0]>0.3:
                tan_val = abs(landmark_position_rel_gt[1][0]/landmark_position_rel_gt[0][0])
                angle = math.atan(min(tan_val,4))
                # if angle < 1.047:
                if angle < 0.47878508936:
                    # standard deviation proportional to the distance to the landmark
                    s_landmark_x = std_dev_landmark_x * landmark_position_rel_gt[0][0]
                    s_landmark_y = std_dev_landmark_y * landmark_position_rel_gt[1][0]

                    # add noise to the ground truth position of the landmark
                    landmark_position_rel_x = landmark_position_rel_gt[0][0] + s_landmark_x * np.random.standard_normal()
                    landmark_position_rel_y = landmark_position_rel_gt[1][0] + s_landmark_y * np.random.standard_normal()

                    # populate the position and covariance of the landmarks
                    msg_landmark = Landmark_Msg()
                    msg_landmark.label = i               
                    msg_landmark.x = landmark_position_rel_x    # x
                    msg_landmark.y = landmark_position_rel_y    # y
                    msg_landmark.s_x = s_landmark_x**2          # s_x^2
                    msg_landmark.s_y = s_landmark_y**2          # s_y^2
                    # add to the landmarks message
                    msg_landmarks.landmarks.append(msg_landmark)

                    marker = self.create_landmark_marker(i, landmark_position_rel_gt[0][0], landmark_position_rel_gt[1][0])
                    marker_array_msg.markers.append(marker)

        if len(msg_landmarks.landmarks) > 0: 
            self.pub_landmarks.publish(msg_landmarks)
            self.pub_landmark_markers.publish(marker_array_msg)
    
    def publish_landmarks_using_subscriber(self):
        rospy.Subscriber('/gazebo/model_states_throttled', ModelStates, self.callback)
        rospy.spin()
 

    def publish_landmarks_using_service(self):
        """Publish landmarks by using the gazebo/get_model_state service.

        The function reads positions of the landmarks relative to the robot and
        checks if they are within the field of view of the robot. All such landmarks
        are published as a Landmarks_Msg

        """
        # Simulation labels: Purple: 0, Orange: 1, Yellow: 2, Blue: 3,
        # Green: 4, Red: 5, Black: 6, Turquoise: 7

        msg_landmarks = Landmarks_Msg()
        marker_array_msg = MarkerArray()

        # In the gazebo/model_states, the cylinders are labelled as obstacle_1 to 8
        for i in range(0, 8):
            landmark_position_rel_gt = self.gazebo_srv('obstacle_' + str(i+1), 'turtlebot3_burger').pose.position

            # check for infront and within field of view
            if landmark_position_rel_gt.x>0:
                tan_val = abs(landmark_position_rel_gt.y/landmark_position_rel_gt.x)
                angle = math.atan(min(tan_val, 4))  # restrict to 75 deg

                # if angle < 1.047:
                if angle < 0.47878508936:   # this is the half fov of the sensor
                    # standard deviation proportional to the distance to the landmark
                    s_landmark_x = std_dev_landmark_x * landmark_position_rel_gt.x
                    s_landmark_y = std_dev_landmark_y * landmark_position_rel_gt.y

                    # add noise to the ground truth position of the landmark
                    landmark_position_rel_x = landmark_position_rel_gt.x + s_landmark_x * np.random.standard_normal()
                    landmark_position_rel_y = landmark_position_rel_gt.y + s_landmark_y * np.random.standard_normal()

                    # populate the position and covariance of the landmarks
                    msg_landmark = Landmark_Msg()
                    msg_landmark.label = i                      # label
                    msg_landmark.x = landmark_position_rel_x    # x
                    msg_landmark.y = landmark_position_rel_y    # y
                    msg_landmark.s_x = s_landmark_x**2          # s_x^2
                    msg_landmark.s_y = s_landmark_y**2          # s_y^2
                    # add to the landmarks message
                    msg_landmarks.landmarks.append(msg_landmark)
                    
                    marker = self.create_landmark_marker(i, landmark_position_rel_gt.x, landmark_position_rel_gt.y)
                    marker_array_msg.markers.append(marker)

        if len(msg_landmarks.landmarks) > 0: 
            self.pub_landmarks.publish(msg_landmarks)
            self.pub_landmark_markers.publish(marker_array_msg)


if __name__ == '__main__':
    rospy.init_node('landmark_publisher')
    publm = PublishLandmarks()

    # When using on a rosbag use this
    #publm.publish_landmarks_using_subscriber()

    # When using with Gazebo use the following
    rospy.wait_for_service('/gazebo/get_model_state')
    while(not rospy.is_shutdown()):
        try:
            gazebo_model_states_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            publm.set_gazebo_service(gazebo_model_states_service)
            publm.publish_landmarks_using_service()
            rospy.sleep(0.1)    # this controls the publising speed
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)