import time
import socket

host = '10.8.216.77'
port = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

while True:
    try:
        s.send(b"Hello\n\0")
        time.sleep(5)
        print("Sending...")
    except KeyboardInterrupt:
        break

s.close()
