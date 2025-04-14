import socket
import struct
import time
import zlib
from PIL import Image
import io

# Configuration
PORT = 9999
MAGIC_NUMBER = 0xf09f93b7
FPS = 30  # Frames per second

# Create a simple blank image
width, height = 320, 240
img = Image.open(r"scripts/irom.png")
# blank_img = Image.new('RGB', (width, height), color='white')
buffer = io.BytesIO()
img.save(buffer, format="JPEG")
image_data = buffer.getvalue()
print(f"Created blank image: {len(image_data)} bytes")

# Create server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('127.0.0.1', PORT))
server_socket.listen(1)
print(f"Server listening on port {PORT}")

try:
    while True:
        print("Waiting for connection...")
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")
        
        try:
            # Main sending loop
            while True:
                # Calculate CRC
                crc = zlib.crc32(image_data) & 0xffffffff
                
                # Send magic number
                client_socket.sendall(struct.pack('<I', MAGIC_NUMBER))
                
                # Send CRC
                client_socket.sendall(struct.pack('<I', crc))
                
                # Send frame size
                client_socket.sendall(struct.pack('<I', len(image_data)))
                
                # Send the image data in one go
                client_socket.sendall(image_data)
                
                print(f"Sent frame: {len(image_data)} bytes")
                
                # Sleep to maintain frame rate
                time.sleep(1.0 / FPS)
                
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"Client disconnected: {e}")
        finally:
            client_socket.close()
            
except KeyboardInterrupt:
    print("Server stopped by user")
finally:
    server_socket.close()