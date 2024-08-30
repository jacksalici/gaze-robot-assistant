import socket
import threading

clients = []

def handle_client(client_socket, client_address):
    while True:
        try:
            message = client_socket.recv(1024).decode('utf-8')
            if message:
                print(f"Received message from {client_address}: {message}")
                broadcast(message, client_socket)
            else:
                remove_client(client_socket)
                break
        except:
            continue

def broadcast(message, sender_socket):
    for client in clients:
        if client != sender_socket:
            try:
                client.send(message.encode('utf-8'))
            except:
                remove_client(client)

def remove_client(client_socket):
    if client_socket in clients:
        clients.remove(client_socket)
        client_socket.close()

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(3)
    print("Bridge server started, waiting for connections...")

    while True:
        client_socket, client_address = server_socket.accept()
        clients.append(client_socket)
        print(f"Client {client_address} connected.")
        client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
        client_thread.start()

if __name__ == "__main__":
    start_server()
