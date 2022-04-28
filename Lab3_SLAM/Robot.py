import numpy as np

from geometry import Relative2AbsolutePose as R2A_P


class Robot:
    def __init__(self, initial,odo, index, measurements, landmarks, covariance):

        self.initial = initial
        self.current = initial

        self.index = index

        assert(len(landmarks)==len(measurements),"Number of measurements should equal number of landmarks!")
        self.measurements = measurements
        self.landmarks = landmarks
        self.odo = odo
        self.covariance = covariance
    
    def get_initial(self):
        # Initialised state
        return self.initial
    
    def get_current(self):
        # Current state
        return self.current

    def get_covariance_processed(self):
        # Covariance matrix
        #INV, SQRT, 
        processed_covariance = np.linalg.inv(np.transpose(np.sqrt(self.covariance)))
        return processed_covariance

    def set_current(self,new):
        self.current = new

    def set_new_measurement(self, measurement):
        self.measurements.append(measurement)

    def set_new_measurement_index(self, index):
        self.landmarks.append(index)
    
    def get_index(self):
        # Index of state
        return self.index
    
    def get_measurements(self):
        # List of relative measurements between landmarks
        return self.measurements
    
    def get_landmarks(self):
        # List of landmark indices
        return self.landmarks
    
    def get_odo(self):
        return self.odo

    def predict_state(self,at):
        return R2A_P(at,self.odo)