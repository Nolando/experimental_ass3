start
    initialise state vector (3 * num robot poses + 2 * num landmark positions, 1)

    # first iteration only
    set prev_pose as [0,0,0] jacobian with covariance sigma_0

    while not done
        for each state
        curr_pose = Relative2AbsolutePose (previous pose, odometry)
        
        if !curr_landmark

            curr_landmark = Relative2AbsoluteLandmark (curr_pose, measurement of the landmark from that pose)


        # Error for the motion/robot position
        [predicted_motion, F_k, G_k] = Absolute2RelativePose (prev_pose, curr_pose)
        residual_motion = predicted_motion - robot.getodo()

        index = robot.getindex
        for row in len(G_k):
            for col in len(row):
                A[index*num_state*3+row,index*num_state*3+col] = G_k[row,col]
                A[(index+1)*num_state*3+row,index*num_state*3+col] = F_k[row,col]
        
        for row in len(residual_motion):
            b[index*num_state*3+row] = residual_motian(row)
        # end motion part

        # Error for the landmark measurements
        for i in each index of landmark (len(robot.getlandmarks))
            temp_landmark = robot.getmeasurement(i)
            [predicted_measurement, H_k, J_j] = Absolute2RelativeLandmark (curr_pose, temp_landmark) 
            residual_measurement = predicted_measurement - initial landmark

            place shit in a and b
