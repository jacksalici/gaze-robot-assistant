import numpy as np

def inv_transformation_matrix(E):
    R = E[:3, :3]
    T = E[:3, -1].reshape((3,1))
    assert R.shape == (3,3) and T.shape == (3, 1)
    return np.hstack((R.T,-R.T@T))
    

import json   
import tomllib
import time

class box:
    def __init__(self, id, position) -> None:
        self.config = tomllib.load(open("config.toml", "rb"))
        self.position = position
        self.id = id
        self.__gazedSince = -1 # -1 if not gazed, second since the started
        
    def isGazed(self, gaze_position) -> bool:
        """check if the box is being gazed.

        Args:
            gaze_position (List): position of the gaze

        Returns:
            bool: return true if it has been gazed for more than the time threshold
        """        
        
        if np.linalg.norm(gaze_position-self.position) < self.config["position_threshold"]:
            if self.__gazedSince < 0:
                self.__gazedSince = time.time
            elif time.time - self.__gazedSince > self.config["gaze_time_min"]:
                return True 
        else:
            self.__gazedSince = -1

        return False

