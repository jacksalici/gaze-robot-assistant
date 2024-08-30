import socket
import threading

def receive_messages(client_socket):
    while True:
        try:
            message = client_socket.recv(1024).decode('utf-8')
            if message:
                print(f"Received: {message}")
        except:
            print("Connection closed.")
            break

def send_messages(client_socket):
    while True:
        message = input("You: ")
        try:
            client_socket.send(message.encode('utf-8'))
        except:
            print("Connection closed.")
            break

def start_client(server_ip):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, 12345))
    print("Connected to the server.")

    receive_thread = threading.Thread(target=receive_messages, args=(client_socket,))
    receive_thread.start()

    send_thread = threading.Thread(target=send_messages, args=(client_socket,))
    send_thread.start()

if __name__ == "__main__":
    server_ip = input("Enter the server's IP address: ")
    start_client(server_ip)
