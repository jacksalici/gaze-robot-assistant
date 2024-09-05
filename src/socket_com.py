import socket
import threading
from cobot_boxes import *

class Client():
    def __init__(self, server_ip):
        self.server_ip = server_ip

    
    def send_message(self, message):

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        server_address = (self.server_ip, 12345)
        client_socket.connect(server_address)

        try:

            client_socket.sendall(message.encode('utf-8'))

        finally:
            client_socket.close()
    

class Server():
    def __init__(self) -> None:
        from ros_robot_controller import RobotController


        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12345))
        server_socket.listen(3)
        print("INFO: Server started.")

        self.robotController = RobotController()

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"INFO: Client {client_address} connected.")
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
            client_thread.start()

    def handle_client(self, client_socket, client_address):
        try:
            print(f"Connection from {client_address}")

            while True:
                # Receive data from the client
                data = client_socket.recv(1024)
                if data:
                    # Decode the message from the client
                    message = data.decode('utf-8')
                    print(f"Received message: {message} from {client_address}")

                    msg = load_CobotSocketMessage(message)
                    print(msg)
                    if msg.init:
                        print("ACTION: INIT")
                        self.robotController.init_boxes(self.trasform_all_coordinates(msg.boxes_position))    

                    if msg.trigger_robot:
                        print("ACTION: TRIGGER")
                        self.robotController.move_to_position(self.trasform_coordinates(*msg.target_position))

                else:
                    break

        finally:
            # Clean up the connection
            client_socket.close()
        
    def remove_client(self, client_socket):
        if client_socket in self.clients:
            self.clients.remove(client_socket)
            client_socket.close()


    def trasform_coordinates(self, x, y, z):
        TABLE_HEIGHT = 0.6
        return [-y, x, TABLE_HEIGHT]

    def trasform_all_coordinates(self, coordinates_list):
        return [self.trasform_all_coordinates(*coordinates) for coordinates in coordinates_list]


if __name__ == "__main__":
    client = Client("0.0.0.0")
    
    from cobot_boxes import *
    
    while True:
        msg = CobotSocketMessage(
                        init=True,
                        trigger_robot=False,
                        glasses_position=[0,0,0],
                        target_position=[0,0,0],
                        boxes_position= [[1,2,3], [2,3,3]]
                    )
                    
        client.send_message(dumps_CobotSocketMessage(msg))
        
        l = input("enter to continue")
