import asyncio
import io
import os
import time
import queue
import threading
import ssl
import numpy as np
from datetime import datetime
from PIL import Image
import websockets
import argparse

class CameraClient:
    def __init__(self, websocket_url, timeout, save_directory='saved_images'):
        """
        Initialize the ESP camera client
        
        Args:
            websocket_url (str): WebSocket URL to connect to
            save_directory (str): Directory to save images
            display (bool): Whether to display images in a window
            save_interval (int): Save every Nth image (to reduce disk usage)
        """
        self.websocket_url = websocket_url
        self.save_directory = save_directory
        self.latest_frame = (None, None)  # (image_bytes, timestamp)
        self.lock_timeout = timeout
        self.frame_lock = threading.Lock()
        self.running = False
        self.total_frames = 0
        self.ws_connection = None
        self.shutdown_event = threading.Event()
    
    def update_latest_frame(self, image_bytes, timestamp):
        """Thread-safe update of the latest frame"""
        if self.frame_lock.acquire(timeout=self.lock_timeout):
            try:
                self.latest_frame = (image_bytes, timestamp)
            finally:
                self.frame_lock.release()
        else:
            print("Warning: Could not acquire frame lock in time for updating")
    
    def get_latest_frame(self):
        """Thread-safe access to the latest frame"""
        if self.frame_lock.acquire(timeout=self.lock_timeout):
            try:
                return self.latest_frame
            finally:
                self.frame_lock.release()
        else:
            print("Warning: Could not acquire frame lock in time retreiving")
            return (None, None)

    async def connect_and_receive(self):
        """Connect to the websocket server and start receiving images"""
        self.running = True

        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.load_verify_locations("cloud-deck/certificates/cacert.pem")
        ssl_context.check_hostname = False
        
        try:
            async with websockets.connect(self.websocket_url, ssl=ssl_context) as websocket:
                print(f"Connected to {self.websocket_url}")
                start_time = time.time()
                
                while self.running and not self.shutdown_event.is_set():
                    try:
                        try:
                            # Use a timeout to check for shutdown regularly
                            image_bytes = await asyncio.wait_for(
                                websocket.recv(), 
                                timeout=0.5
                            )
                            timestamp = datetime.now()

                            # Process frame metrics
                            self.total_frames += 1
                            elapsed = time.time() - start_time
                            fps = self.total_frames / elapsed if elapsed > 0 else 0
                            
                            # Put the image in the queue for saving (if it's an interval frame)
                            self.update_latest_frame(image_bytes, timestamp)
                            
                            # Print occasional status
                            if self.total_frames % 30 == 0:
                                print(f"Received {self.total_frames} frames, FPS: {fps:.1f}")

                        except asyncio.TimeoutError:
                            # This is expected - just use it to check if we should shut down
                            if self.shutdown_event.is_set():
                                break
                            continue
                            
                    except websockets.exceptions.ConnectionClosed:
                        print("WebSocket connection closed")
                        break
                    except asyncio.CancelledError:
                        print("Receiving task cancelled")
                        break
                    except Exception as e:
                        print(f"Error receiving image: {e}")
                        if str(e) == "no current event loop":
                            break
                        
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            self.running = False
            print("Client stopped")


    async def shutdown(self):
        """Shut down the client gracefully"""
        print("Shutting down client...")
        self.shutdown_event.set()
        self.running = False
        
        # If websocket is still open, close it properly
        if self.ws_connection:
            try:
                await self.ws_connection.close()
            except Exception as e:
                print(f"Error closing websocket: {e}")
            
        print(f"Shutdown complete.")

def main():
    parser = argparse.ArgumentParser(description="ESP32 Camera Client")
    parser.add_argument("host", type=str, help="Host IP address")
    parser.add_argument("--timeout", type=float, default=1.0, help="Lock timeout in seconds")
    args = parser.parse_args()

    # Construct the websocket URL
    websocket_url = f"wss://{args.host}/ws"

    # Create the camera client
    cam_client = CameraClient(websocket_url, args.timeout)

    # Start the camera stream
    try:
        asyncio.run(cam_client.connect_and_receive())
    except KeyboardInterrupt:
        print("Camera stream interrupted")
    except Exception as e:
        print(f"Camera stream error: {e}")
    finally:
        asyncio.run(cam_client.shutdown())

if __name__ == "__main__":
    main()