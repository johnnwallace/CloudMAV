import socket

def main():
    host = '10.8.216.77'  # Change this to the server's address
    port = 5000        # Change this to the server's port

    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((host, port))
            print(f"Connected to {host}:{port}")
            
            # Optionally, send some data
            message = "Hello, Server!"
            s.sendall(message.encode())
            print(f"Sent: {message}")
            
            # Optionally, receive data
            # data = s.recv(1024)
            # print(f"Received: {data.decode()}")
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()