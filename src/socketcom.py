import socket
import threading


class Client():
    def __init__(self, server_ip):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, 12345))
        print("Connected to the server.")

        receive_thread = threading.Thread(target=self.receive_messages, args=(client_socket,))
        receive_thread.start()

        send_thread = threading.Thread(target=self.send_messages, args=(client_socket,))
        send_thread.start()

    def receive_messages(self, client_socket):
        while True:
            try:
                message = client_socket.recv(1024).decode('utf-8')
                if message:
                    print(f"Received: {message}")
            except:
                print("Connection closed.")
                break

    def send_messages(self, client_socket):
        while True:
            message = input("You: ")
            try:
                client_socket.send(message.encode('utf-8'))
            except:
                print("Connection closed.")
                break

class Server():
    def __init__(self) -> None:
        self.clients = []

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12345))
        server_socket.listen(3)
        print("INFO: Server started.")

        while True:
            client_socket, client_address = server_socket.accept()
            self.clients.append(client_socket)
            print(f"Client {client_address} connected.")
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
            client_thread.start()

    def handle_client(self, client_socket, client_address):
        while True:
            try:
                message = client_socket.recv(1024).decode('utf-8')
                if message:
                    print(f"INFO: Received message from {client_address}: {message}")
                    self.broadcast(message, client_socket)
                else:
                    self.remove_client(client_socket)
                    break
            except:
                continue

    def broadcast(self, message, sender_socket):
        for client in self.clients:
            if client != sender_socket:
                try:
                    client.send(message.encode('utf-8'))
                except:
                    self.remove_client(client)

    def remove_client(self, client_socket):
        if client_socket in self.clients:
            self.clients.remove(client_socket)
            client_socket.close()


    
if __name__ == "__main__":
    client = Client("0.0.0.0")
