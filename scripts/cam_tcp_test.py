import socket
import select
import struct
import cv2
import zlib
import numpy as np
import time
import threading
import queue
import argparse

RECV_BUFFER_SIZE = 1024 * 1024
MAX_FRAME_SIZE = 800 * 600 * 3 * 5 / 8
CHUNK_SIZE = 4096
MAGIC_BYTES = bytearray([0xf0, 0x9f, 0x93, 0xb7])

class CameraClient:
    def __init__(self, host, port, buffer_size=5):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.running = False
        
        # Frame buffer
        self.frame_queue = queue.Queue(maxsize=buffer_size)
        self.latest_frame = None
        
        # Statistics
        self.stats = {
            'frames_received': 0,
            'bytes_received': 0,
            'fps': 0,
            'latency_ms': 0,
            'start_time': 0
        }
    
    def connect(self):
        """Connect to the camera server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECV_BUFFER_SIZE)
            self.sock.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the camera server"""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.connected = False
        print("Disconnected from server")
    
    def receive_frames(self):
        """Background thread to receive frames"""
        self.running = True
        self.stats['start_time'] = time.time()
        
        magic_index = 0
        while self.running and self.connected:
            try:
                # Wait for bytes
                readable, _, _ = select.select([self.sock], [], [], None)

                # Sliding window for magic bytes seems to add ~3 ms to latency
                if magic_index < len(MAGIC_BYTES):
                    # Still looking for magic bytes
                    print("Reading magic")
                    byte = self.sock.recv(1)
                    print(byte)
                    if not byte:
                        self.disconnect()
                        break

                    if byte[0] == MAGIC_BYTES[magic_index]:
                        # Matched next byte in sequence
                        magic_index += 1
                        if magic_index == len(MAGIC_BYTES):
                            # Complete magic number found!
                            print("Magic bytes detected!")
                    else:
                        # Mismatch - check if this byte could be the start of magic
                        if byte == MAGIC_BYTES[0]:
                            magic_index = 1  # Started a new potential match
                        else:
                            magic_index = 0  # Reset magic detection
                else:

                    # Read CRC
                    print("Reading CRC")
                    crc_data = self.sock.recv(4)
                    print(crc_data)
                    if not crc_data or len(crc_data) != 4:
                        self.disconnect()
                        break
                    crc = struct.unpack('I', crc_data)[0]
                    print(f"CRC: {crc}")
                    self.stats['bytes_received'] += 4

                    # Read frame size and verify
                    print("Reading size")
                    size_data = self.sock.recv(4)
                    if not size_data or len(size_data) != 4:
                        print("Bad length!")
                        self.disconnect()
                        break
                    frame_size = struct.unpack('I', size_data)[0]
                    if frame_size > MAX_FRAME_SIZE:
                        print(f"Bad length!: {frame_size}")
                        continue
                    print(f"Frame size: {frame_size}")
                    self.stats['bytes_received'] += 4
                    
                    # Read frame data
                    frame_data = bytearray()
                    remaining = frame_size
                    
                    while remaining > 0:
                        chunk_size = min(remaining, CHUNK_SIZE)
                        print("Reading chunk")
                        chunk = self.sock.recv(chunk_size)
                        
                        if not chunk:
                            self.disconnect()
                            break
                        
                        frame_data.extend(chunk)
                        remaining -= len(chunk)
                    
                    # Verify CRC
                    if zlib.crc32(frame_data) & 0xffffffff != crc:
                        print("Bad CRC!")
                        continue

                    magic_index = 0
                    if len(frame_data) == frame_size:
                        # Successfully received a complete frame
                        # Decode JPEG to image
                        frame_timestamp = time.time()
                        img = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                        
                        if img is not None:
                            # Add timestamp to the frame for latency calculation
                            frame_with_ts = {
                                'image': img,
                                'timestamp': frame_timestamp
                            }
                            
                            # Put in queue, if full, remove oldest
                            if self.frame_queue.full():
                                try:
                                    self.frame_queue.get_nowait()
                                except queue.Empty:
                                    pass
                            
                            self.frame_queue.put(frame_with_ts)
                            self.latest_frame = frame_with_ts
                            
                            # Update statistics
                            self.stats['frames_received'] += 1
                            self.stats['bytes_received'] += frame_size
                            
                            elapsed = frame_timestamp - self.stats['start_time']
                            if elapsed > 0:
                                self.stats['fps'] = self.stats['frames_received'] / elapsed
                
            except Exception as e:
                print(f"Error receiving frame: {e}")
                self.connected = False
                break
        
        self.running = False
    
    def display_frames(self):
        """Display received frames and handle user input"""
        cv2.namedWindow('ESP32 Camera Stream', cv2.WINDOW_NORMAL)
        last_stats_time = time.time()
        
        while self.running:
            try:
                # Get frame with timeout
                frame_data = self.frame_queue.get(timeout=1.0)
                
                # Calculate and display latency
                latency = (time.time() - frame_data['timestamp']) * 1000
                self.stats['latency_ms'] = latency
                
                # Add stats to the frame
                img = frame_data['image'].copy()
                stats_text = f"FPS: {self.stats['fps']:.1f}, Latency: {latency:.1f}ms"
                cv2.putText(img, stats_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show the frame
                cv2.imshow('ESP32 Camera Stream', img)
                
                # Show statistics periodically
                if time.time() - last_stats_time >= 5.0:
                    total_mb = self.stats['bytes_received'] / (1024 * 1024)
                    elapsed = time.time() - self.stats['start_time']
                    mbps = total_mb / elapsed
                    
                    print(f"Statistics: {self.stats['frames_received']} frames received, "
                          f"FPS: {self.stats['fps']:.1f}, "
                          f"Latency: {self.stats['latency_ms']:.1f}ms, "
                          f"Data rate: {mbps:.2f} MB/s")
                    
                    last_stats_time = time.time()
                
                # Check for exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
            except queue.Empty:
                # No frames available, check if we're still running
                if not self.running or not self.connected:
                    break
        
        cv2.destroyAllWindows()
    
    def start(self):
        """Start the client"""
        if not self.connect():
            return False
        
        # Start receiver thread
        self.receiver_thread = threading.Thread(target=self.receive_frames)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        # Display frames (this will block until user quits)
        self.display_frames()
        
        # Clean up
        self.disconnect()
        return True

def main():
    parser = argparse.ArgumentParser(description='ESP32 Camera Client')
    parser.add_argument('--host', default='192.168.1.168', help='ESP32 IP address')
    parser.add_argument('--port', type=int, default=8080, help='Camera server port')
    args = parser.parse_args()
    
    client = CameraClient(args.host, args.port)
    client.start()

if __name__ == "__main__":
    main()