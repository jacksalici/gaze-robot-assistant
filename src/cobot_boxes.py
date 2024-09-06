import numpy as np
import cv2

def inv_transformation_matrix(E):
    R = E[:3, :3]
    T = E[:3, -1].reshape((3,1))
    assert R.shape == (3,3) and T.shape == (3, 1)
    return np.hstack((R.T,-R.T@T))

import json   
import toml
import time

class Box:
    def __init__(self, id, positionInRobotFrame = None, rotationInRobotFrame = None, positionInGlassesFrame = None, rotationInGlassesFrame = None) -> None:
        self.config = toml.load("config.toml")
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
        
    def isGazed(self, gaze_position_in_robot_frame) -> bool:
        """check if the box is being gazed.

        Args:
            gaze_position (List): position of the gaze

        Returns:
            bool: return true if it has been gazed for more than the time threshold
        """        
        
        #norm excludes Z axis
        if np.linalg.norm(gaze_position_in_robot_frame[:2]-self.positionInRobotFrame[:2]) < self.config["position_threshold"]:
            if self.__gazedSince < 0:
                self.__gazedSince = time.time()
            elif time.time() - self.__gazedSince > self.config["gaze_time_min"]:
                print(f"INFO: Box with id {self.id} is being gazed!")
                self.__gazedSince = -1
                return True 
        else:
            self.__gazedSince = -1

        return False
    

class Cobot(Box):
    def __init__(self, id, positionInRobotFrame=None, rotationInRobotFrame=None, positionInGlassesFrame=None, rotationInGlassesFrame=None) -> None:
        super().__init__(id, positionInRobotFrame, rotationInRobotFrame, positionInGlassesFrame, rotationInGlassesFrame)
        self.__setExtrinsic()

    
    def updatePosition(self, positionInGlassesFrame, rotationInGlassesFrame):
        self.positionInGlassesFrame = positionInGlassesFrame 
        self.rotationInGlassesFrame = rotationInGlassesFrame 
        self.__setExtrinsic()
        
    def __setExtrinsic(self):
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
    
def updateCobot(cobot: Cobot, marker_ids, rvecs, tvecs):
    for i, id in enumerate(marker_ids):
        if id[0] == cobot.id:
            cobot.updatePosition(
                positionInGlassesFrame = tvecs[i],
                rotationInGlassesFrame = rvecs[i]
            )
           
            
from dataclasses import dataclass, asdict, make_dataclass
import json
from typing import List

#todo: as for now the box are points and trigger robot target is the same point.
@dataclass
class CobotSocketMessage: # one message for both action (spawn boxes or trigger robot)
    init: bool # when true, spawn boxes
    trigger_robot: bool # when true, trigger robot
    target_position: List[float] # goto position for the robot to a box
    glasses_position: List[float] # goto position for the robot when has to bring sth to the person
    boxes_position: List[List[float]] # position of all the boxes, useful just when trigging
    boxes_yaws: List[float]
    
def dumps_CobotSocketMessage(msg: CobotSocketMessage) -> str:
    return json.dumps(asdict(msg))

def load_CobotSocketMessage(msg: str) -> CobotSocketMessage:
    my_dict = json.loads(msg)
    return make_dataclass(
    "CobotSocketMessage", ((k, type(v)) for k, v in my_dict.items())
)(**my_dict)
    
    