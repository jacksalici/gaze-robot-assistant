from socket_com import Server
from cobot_boxes import *
from ros_robot_controller import RobotController
import time


def trasform_coordinates(x, y, z):
    TABLE_HEIGHT = 0.4
    return [-y, x, TABLE_HEIGHT]

def trasform_all_coordinates(coordinates_list):
    return [trasform_all_coordinates(*coordinates) for coordinates in coordinates_list]

def logging(str_msg):
    robotController = RobotController()
    print ("INFO: new message received", str_msg)
    
    msg = load_CobotSocketMessage(str_msg)
    if msg.init:
        print("ACTION: INIT")
        robotController.init_boxes(trasform_all_coordinates(msg.boxes_position))    

    if msg.trigger_robot:
        print("ACTION: TRIGGER")
        robotController.move_to_position(trasform_coordinates(*msg.target_position))

if __name__ == "__main__":
    server = Server(logging)
