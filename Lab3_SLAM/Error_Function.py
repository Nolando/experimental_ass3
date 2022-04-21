"""
Updated on Tue Apr 07 2020
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams, Tejaswi Digumarti
"""

import numpy as np
from geometry import RelativeLandmarkPositions 
from geometry import pi2pi


def ErrorFunction(solution_file, gt_file):
    """
    Calculates the error between ground truth data and the data obtained after SLAM
    :param solution_file: a .txt file containing the absolute positions of the landmarks as computed after SLAM
    :param gt_file: a .txt file containing the ground truth absolute positions of the landmarks
    :return : relative ground truth landmark positions - relative estimated landmark positions
    """
   
    landmark_pred = []
    landmark_gt = []

    # parse the txt files
    if solution_file.endswith('.txt') and gt_file.endswith('.txt'):
        f = open(solution_file, 'r')
        for line in f:
            info = line.split(' ')
            if len(info) > 1 and info[1] == 'POINT2D':
                landmark_pred.append([info[2], info[3], info[4]])
        f.close()
           
        f = open(gt_file, 'r')
        for line in f:
            info = line.split(' ')
            if len(info) > 1 and info[1] == 'POINT2D':
                landmark_gt.append([info[2], info[3], info[4]])
        f.close()

    # Compute positions of landmarks relative to the positions of their previous landmarks
    landmark_error = []
    landmark_pred = sorted(landmark_pred)
    for i in range(0, len(landmark_pred) - 1):
        landmarks_pred_rel = RelativeLandmarkPositions(landmark_pred[i], landmark_pred[i+1])
        landmark1 = landmark_pred[i]
        label1 = int(landmark1[0])
        landmark2 = landmark_pred[i+1]
        label2 = int(landmark2[0])
        # GT file should be ordered by cylinders label 
        # e.g.
        # POINT2D 1 x1 y1
        # POINT2D 2 x2 y2
        # '
        # '
        # '
        landmarks_gt_rel = RelativeLandmarkPositions(landmark_gt[label1-1], landmark_gt[label2-1])
        landmark_error.append(np.array(landmarks_gt_rel) - np.array(landmarks_pred_rel))
    
    if len(landmark_pred) > 1:
        error_landmark = (1.0/(len(landmark_pred)-1))*np.linalg.norm(landmark_error)
    else:
        error_landmark = 0.
    # print 'Your Solution Error', errorLandmark
    
    return error_landmark


if __name__ == "__main__":
    
    """
    For self testing only
    """

    file_solution = 'SLAM_Output.txt'
    file_gt = 'gt.txt'
    error = ErrorFunction(file_solution, file_gt)
    

