from socket_com import Server
from common import *

def ciao(str_msg):
    msg = load_CobotSocketMessage(str_msg)
    if msg.init:
        print("ACTION: INIT")
    if msg.trigger_robot:
        print("ACTION: TRIGGER")
    print (str_msg)

if __name__ == "__main__":
    server = Server(ciao)
