import numpy as np
import cv2

def inv_transformation_matrix(E):
    R = E[:3, :3]
    T = E[:3, -1].reshape((3,1))
    assert R.shape == (3,3) and T.shape == (3, 1)
    return np.hstack((R.T,-R.T@T))

import json   
import tomllib
import time

class Box:
    def __init__(self, id, positionInRobotFrame = None, rotationInRobotFrame = None, positionInGlassesFrame = None, rotationInGlassesFrame = None) -> None:
        self.config = tomllib.load(open("config.toml", "rb"))
        self.id = id
        self.__gazedSince = -1 # -1 if not gazed, second since the started
        
        self.positionInRobotFrame = positionInRobotFrame 
        self.rotationInRobotFrame = rotationInRobotFrame 

        self.positionInGlassesFrame = positionInGlassesFrame 
        self.rotationInGlassesFrame = rotationInGlassesFrame 
    
    def setPositionInRobotFrame(self, position):
        self.positionInRobotFrame = position
    
    def getPositionInRobotFrame(self):
        return self.positionInRobotFrame
        
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
    

class Cobot(Box):
    def __init__(self, id, positionInRobotFrame=None, rotationInRobotFrame=None, positionInGlassesFrame=None, rotationInGlassesFrame=None) -> None:
        super().__init__(id, positionInRobotFrame, rotationInRobotFrame, positionInGlassesFrame, rotationInGlassesFrame)
        
        R = cv2.Rodrigues(np.array(self.rotationInGlassesFrame))[0]

        self.E_robot2glasses = inv_transformation_matrix(np.hstack((R, self.positionInGlassesFrame.reshape((3,1)))))
    
    
    def trasformInRobotFrame(self, positionInGlassesFrame):
        return (self.E_robot2glasses
                    @ np.append(positionInGlassesFrame, [1])
                )[:3]
    
    def trasformInNewRobotFrame(self, positionInGlassesFrame, robotPositionInGlassesFrame, robotRotationInGlassesFrame):
        return (inv_transformation_matrix(np.hstack((cv2.Rodrigues(np.array(robotRotationInGlassesFrame))[0], robotPositionInGlassesFrame))) 
                    @ np.append(positionInGlassesFrame, [1])
                )[:3]
       
from typing import Dict
def generateBoxes(marker_ids, robot_id, rvecs, tvecs):
    cobot: Cobot = None
    boxes: Dict[int, Box] = {} 

    for i, id in enumerate(marker_ids):
        if id[0] == robot_id:
            cobot = Cobot(id[0],
                          positionInRobotFrame=np.array([0, 0, 0]),
                          rotationInRobotFrame=np.array([0, 0, 0]),
                          positionInGlassesFrame=tvecs[i],
                          rotationInGlassesFrame=rvecs[i]
                          )
        else:
            boxes[id[0]] = Box(id[0], positionInGlassesFrame=tvecs[i], rotationInGlassesFrame=rvecs[i])
            
            
    for id, box in boxes.items():
        if id != robot_id:
            box.setPositionInRobotFrame(cobot.trasformInRobotFrame(box.positionInGlassesFrame))
               
                
    print("INFO: Boxes generated")
    
    return boxes, cobot
    
    