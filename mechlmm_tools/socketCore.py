import socket
import threading
import time

# IP and port for Unity
HOST = '127.0.0.1'  # Localhost or specify your IP
PORT = 8080  # Make sure the port matches the Unity listener

# Create socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Function to send data to Unity
def send_data():
    while True:
        message = input("Enter message to send to Unity: ")
        sock.sendall(message.encode())
        time.sleep(1)  # Delay between messages (customize as needed)

# Function to listen for incoming data from Unity
def listen_data():
    while True:
        data = sock.recv(1024).decode()
        if data:
            print(f"Received from Unity: {data}")

# Start threads for sending and receiving data
send_thread = threading.Thread(target=send_data)
listen_thread = threading.Thread(target=listen_data)

send_thread.start()
listen_thread.start()

# Join threads to main thread to run continuously
send_thread.join()
listen_thread.join()
