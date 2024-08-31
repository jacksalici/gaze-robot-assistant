import socket
import threading
from typing import Callable

class Client():
    def __init__(self, server_ip):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((server_ip, 12345))
        print("Connected to the server.")

        receive_thread = threading.Thread(target=self.receive_messages)
        receive_thread.start()

    def receive_messages(self):
        while True:
            try:
                message = self.client_socket.recv(1024).decode('utf-8')
                if message:
                    print(f"Received: {message}")
            except:
                print("Connection closed.")
                break

    def send_message(self, message):
       
        try:
            self.client_socket.send(message.encode('utf-8'))
        except:
            print("ERROR: Socket connection closed.")
            

class Server():
    def __init__(self, function) -> None:
        self.clients = []

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12345))
        server_socket.listen(3)
        print("INFO: Server started.")

        while True:
            client_socket, client_address = server_socket.accept()
            self.clients.append(client_socket)
            print(f"INFO: Client {client_address} connected.")
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address, function))
            client_thread.start()

    def handle_client(self, client_socket, client_address, function: Callable = None):
        while True:
            try:
                message = client_socket.recv(1024).decode('utf-8')
                if message:
                    print(f"INFO: Received message from {client_address}")
                    if function != None:
                        function(message)
                    
                else:
                    self.remove_client(client_socket)
                    break
            except:
                continue

    def remove_client(self, client_socket):
        if client_socket in self.clients:
            self.clients.remove(client_socket)
            client_socket.close()


    
if __name__ == "__main__":
    client = Client("0.0.0.0")
    while True:
        client.send_message(input("Input: "))
