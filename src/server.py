from socket_com import Server
from common import *

def ciao(mex):
    c = load_CobotSocketMessage(mex)
    print (c.boxes_position)

if __name__ == "__main__":
    server = Server(ciao)
