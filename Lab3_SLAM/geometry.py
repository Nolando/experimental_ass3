import numpy as np


def pi2pi(angle):
    """
    Maps angle to the range of [-pi, pi]
    :param angle: then angle that needs to be mapped to the range [-pi, pi]
    :return : angle in the range [-pi, pi]
    """
    dp = 2*np.pi
    if angle <= -dp or angle >= dp:
        angle = angle % dp
    if angle >= np.pi:
        angle = angle - dp
    if angle <= -np.pi:
        angle = angle + dp
    return angle


def Relative2AbsoluteXY(robot_pose_abs, landmark_position_rel):
    """
    Convert's a landmark's position from the robot's frame of reference to the absolute frame of reference
    :param robot_pose_abs: pose of the robot in the the absolute frame of reference [x, y, theta]
    :param landmark_position_rel: position of the landmark in the robot's frame of reference [x, y]
    :return : [position of the landmark in the absolute frame of reference [x, y], H1, H2]
    """
    x1 = robot_pose_abs[0]
    y1 = robot_pose_abs[1]
    theta1 = robot_pose_abs[2]
    x2 = landmark_position_rel[0]
    y2 = landmark_position_rel[1]

    
    landmark_position_rel_vec = [[x2], [y2], [1]]
    
    # R is the transition matrix to robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    # Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -x2*np.sin(theta1)-y2*np.cos(theta1)],
          [0, 1,  x2*np.cos(theta1)-y2*np.sin(theta1)]]
         
    # Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), -np.sin(theta1)],
          [np.sin(theta1),  np.cos(theta1)]]
         
    landmark_abs = np.dot(R, landmark_position_rel_vec) + robot_pose_abs.reshape(3,1)

    return landmark_abs[:2], H1, H2


def Relative2AbsolutePose(robot_pose_abs, u_rel):
    """
    Calculates the new pose of the robot given its current pose in the
    absolute coordinate frame and a motion input.

    :param robot_pose_abs: current pose of the robot in the absolute reference
    frame [x, y, theta]
    :type robot_pose_abs: [type]
    :param u_rel: motion command in the robot's frame of reference
    [dx, dy, dtheta]
    :type u_rel: [type]
    :return: pose of the robot in the absolute reference frame after applying
    the motion and the Jacobians of the new pose wrt, the current pose
    and the motion command respectively
    :rtype: tuple
    """
    #x1 = robot_pose_abs[0][0]
    #y1 = robot_pose_abs[1][0]
    theta1 = robot_pose_abs[2]
    dx = u_rel[0]
    dy = u_rel[1]
    dtheta = u_rel[2]

    # R is the transition matrix of robot frame
    # i.e. X_t+1 = X_t + R(theta_t) * u
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]

    next_robot_pose_abs = np.dot(R, u_rel).reshape(3,1) + robot_pose_abs.reshape(3,1)
    next_robot_pose_abs[2] = pi2pi(next_robot_pose_abs[2])

    # Calculate Jacobian of X_t+1 with respect to the current robot pose X_t
    H1 = [[1, 0, -dx*np.sin(theta1)-dy*np.cos(theta1)],
          [0, 1,  dx*np.cos(theta1)-dy*np.sin(theta1)],
          [0, 0, 1]]

    # Calculate Jacobian of X_t+1 with respect to motion command u
    H2 = [[np.cos(theta1), -np.sin(theta1), 0],
          [np.sin(theta1), np.cos(theta1), 0],
          [0, 0, 1]]

    return next_robot_pose_abs, np.array(H1), np.array(H2)


def Absolute2RelativeXY(robot_pose_abs, landmark_position_abs):
    """Converts a landmark's position from the absolute frame of reference to
    the robot's frame of reference, i.e the position of the landmarks as
    measured by the robot.

    :param robot_pose_abs: pose of the robot in the absolute frame of
    reference [x, y, theta]. 3x1 array
    :type robot_pose_abs: np.array
    :param landmark_position_abs: position of the landmark in the absolute 
    frame of reference [x, y]. 2x1 array
    :type landmark_position_abs: np.array
    :return: position of the landmark in the to the robot's frame of reference
    [x, y], and the Jacobians of
    :rtype: tuple
    """
    x1 = robot_pose_abs[0]
    y1 = robot_pose_abs[1]
    theta1 = robot_pose_abs[2]
    x2 = landmark_position_abs[0]
    y2 = landmark_position_abs[1]

    # Calculate the difference with respect to world frame
    diff = [[x2-x1],
            [y2-y1],
            [1]]

    # R is the transition matrix to robot frame
    R = [[np.cos(-theta1), -np.sin(-theta1), 0],
         [np.sin(-theta1), np.cos(-theta1), 0],
         [0, 0, 1]]

    landmark_position_rel = np.dot(R, diff)

    # Calculate Jacobian of the relative landmark position wrt. the robot pose,
    # i.e. [x1, y1, theta1]
    H1 = [[-np.cos(theta1), -np.sin(theta1), -(x2-x1)*np.sin(theta1)+(y2-y1)*np.cos(theta1)],
          [np.sin(theta1), -np.cos(theta1), -(x2-x1)*np.cos(theta1)-(y2-y1)*np.sin(theta1)]]

    # Calculate Jacobian of the relative landmark position wrt. the absolute
    # landmark pose. i.e. [x2, y2]
    H2 = [[np.cos(theta1), np.sin(theta1)],
          [-np.sin(theta1), np.cos(theta1)]]

    return np.array([[landmark_position_rel[0]], [landmark_position_rel[1]]]), H1, H2


def RelativeLandmarkPositions(landmark_position_abs, next_landmark_position_abs):
    """
    Given two input landmark positions in the absolute frame of reference, computes the relative position of the
    next landmark with respect to the current landmark
    :param landmark_position_abs: position of the current landmark in the absolute reference frame [x, y]
    :param next_landmark_position_abs: position of the next landmark in the absolute reference frame [x, y]
    :return : relative position of the next landmark with respect to the current landmark's position [dx, dy]
    """
    # label is in position [0], hence use positions [1] and [2]
    x1 = float(landmark_position_abs[1])
    y1 = float(landmark_position_abs[2])
    x2 = float(next_landmark_position_abs[1])
    y2 = float(next_landmark_position_abs[2])
    
    # Calculate the difference of position in world frame
    diff = [x2-x1, y2-y1]
    
    return diff
