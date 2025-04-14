import logging
import time
import asyncio
import threading
import signal
import sys
import argparse

import numpy as np
import cv2
from io import BytesIO
from PIL import Image

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

from cameraclient import CameraClient
from pynput import keyboard

def process_frame():
    image_bytes, timestamp = cam_client.get_latest_frame()

    if image_bytes is not None:
        pil_image = Image.open(BytesIO(image_bytes))
        frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        
        # Only display the frame if show_camera is True
        cv2.imshow("WebSocket Stream", frame)
        key = cv2.waitKey(1)
        
        time.sleep(0.1)

    return timestamp

# Create a shutdown event
shutdown_event = threading.Event()

# Start camera websocket client
cam_client = CameraClient(f"wss://192.168.1.168/ws", 1)

def cam_stream():
    print("Starting camera stream...")
    try:
        # Run the client
        asyncio.run(cam_client.connect_and_receive())
    except KeyboardInterrupt:
        print("Camera stream interrupted")
    except Exception as e:
        print(f"Camera stream error: {e}")
    finally:
        print("Camera stream stopped")

def signal_handler(sig, frame):
    print("Shutdown requested...")
    shutdown_event.set()
    cam_client.shutdown_event.set()
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)

# Start camera thread
cam_thread = threading.Thread(target=cam_stream, daemon=True)
cam_thread.start()

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logging.getLogger('websockets').setLevel(logging.WARNING)

# Initialize drivers
cflib.crtp.init_drivers()

# URI to the Crazyflie to connect to
# uri = 'radio://0/1/2M/E7E7E7E7E7'  # Change this to match your Crazyflie
uri = "tcp://192.168.1.168:8000"

# Global control variables
keep_flying = True
current_command = None
command_lock = threading.Lock()

def on_press(key):
    """Called when a key is pressed"""
    global current_command, keep_flying
    
    try:
        # Get the character from the key
        k = key.char.lower()
        
        # Only process valid commands
        if k in ['w', 'a', 's', 'd', 'r', 'f', 'q']:
            with command_lock:
                current_command = k
            
            # Print feedback
            if k == 'w':
                print("Command: Forward")
            elif k == 's':
                print("Command: Backward")
            elif k == 'a':
                print("Command: Left")
            elif k == 'd':
                print("Command: Right")
            elif k == 'r':
                print("Command: Up")
            elif k == 'f':
                print("Command: Down")
            elif k == 'q':
                print("Command: Quit")
                keep_flying = False
                # Stop listener
                return False
    except AttributeError:
        # Special keys like shift, ctrl, etc. will be ignored
        pass

def on_release(key):
    """Called when a key is released"""
    global current_command
    
    try:
        # If the released key is the one currently being processed, clear the command
        k = key.char.lower()
        if k in ['w', 'a', 's', 'd', 'r', 'f', 'q']:
            with command_lock:
                if current_command == k:
                    current_command = None
    except AttributeError:
        # Special keys like shift, ctrl, etc. will be ignored
        pass

def main():
    global keep_flying, current_command
    
    # Start the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    # Initialize and connect to the Crazyflie
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            # Allow some time for the connection to stabilize
            time.sleep(1)
            
            cv2.namedWindow("WebSocket Stream", cv2.WINDOW_NORMAL)

            # Initialize the MotionCommander
            with MotionCommander(scf, default_height=0.5) as mc:
                print("Crazyflie ready! Use w/a/s/d to control, r/f for up/down, q to quit.")
                
                # Main control loop
                while keep_flying:
                    timestamp = process_frame()
                    # Process any current command
                    with command_lock:
                        cmd = current_command
                    
                    if cmd == 'w':
                        mc.forward(0.2)
                    elif cmd == 's':
                        mc.back(0.2)
                    elif cmd == 'a':
                        mc.left(0.2)
                    elif cmd == 'd':
                        mc.right(0.2)
                    elif cmd == 'r':
                        mc.up(0.2)
                    elif cmd == 'f':
                        mc.down(0.2)
                    
                    # Small delay to prevent overwhelming the Crazyflie
                    time.sleep(0.1)
    
    except Exception as e:
        print(f"Error in main Crazyflie control: {e}")
    finally:
        # Clean up
        keep_flying = False
        listener.stop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Handle Ctrl+C
        print("\nProgram terminated by user")
    finally:
        # Clean up
        keep_flying = False