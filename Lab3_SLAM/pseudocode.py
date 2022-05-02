start
    initialise state vector (3 * num robot poses + 2 * num landmark positions, 1)

    # first iteration only
    set prior pose as [0,0,0] jacobian with covariance sigma_0

    # while not done
    loop through each state

        curr_pose = Relative2AbsolutePose (previous pose, odometry)
        
        if !curr_landmark

            curr_landmark = Relative2AbsoluteLandmark (curr_pose, measurement of the landmark from that pose)\

        
        