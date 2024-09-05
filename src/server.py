from socket_com import Server
from cobot_boxes import *
#from ros_robot_controller import RobotController

def initGazebo():
    pass

def triggerRobot():
    pass

#robotController = RobotController()


def logging(str_msg):
    #global robotController
    
    msg = load_CobotSocketMessage(str_msg)
    if msg.init:
        print("ACTION: INIT")
        #robotController.launch_simulation(msg.boxes_position)
    if msg.trigger_robot:
        print("ACTION: TRIGGER")
    print (str_msg)

if __name__ == "__main__":
    server = Server(logging)
