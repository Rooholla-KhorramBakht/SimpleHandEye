import numpy as np

class BasePoseInterface:
    def __init__(self):
        pass
        self.pose = np.eye(4)
        self.T = None   
        self.stamp = None
        
    def getPose(self):
        '''
        return: 4x4 transformation matrix alongside the timestamp
        '''
        raise NotImplementedError