from geometry import Absolute2RelativeXY as A2R_XY
import numpy as np

class Landmark:

    def __init__(self, initial, index, covariance):


        self.initial = initial
        self.current = initial

        self.index = index
        self.covariance = covariance
    
    def get_index(self):
        return self.index
    
    def get_initial(self):
        return self.initial

    def get_covariance_processed(self):
        # Covariance matrix
        processed_covariance = np.linalg.inv(np.transpose(np.sqrt(self.covariance)))
        return processed_covariance

    def set_current(self, new):
        self.current = new
    
    def get_current(self):
        return self.current
    
    def landmark_jacobs(self, state):
        
        return A2R_XY(state.get_current(),self.get_current())
    
