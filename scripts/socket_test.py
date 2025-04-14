import time
import socket

host = '10.9.146.64'
# host = '192.168.1.168'
port = 80

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

while True:
    try:
        s.send(b"Hello\n\0")
        time.sleep(2)
        print("Sending...")
    except KeyboardInterrupt:
        break

s.close()
