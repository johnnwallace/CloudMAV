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

# Parse command line arguments
parser = argparse.ArgumentParser(description='Crazyflie control script with camera stream')
parser.add_argument('--show-camera', action='store_true', 
                    help='Show camera stream window (default: False)')
args = parser.parse_args()

def process_frame():
    image_bytes, timestamp = cam_client.get_latest_frame()

    if image_bytes is not None:
        pil_image = Image.open(BytesIO(image_bytes))
        frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        
        # Only display the frame if show_camera is True
        if args.show_camera:
            cv2.imshow("WebSocket Stream", frame)
            key = cv2.waitKey(1)
        
        time.sleep(0.1)

    return timestamp

HOST_IP = '192.168.1.168'

# Create a shutdown event
shutdown_event = threading.Event()

# Start camera websocket client
cam_client = CameraClient(f"wss://{HOST_IP}/ws", 1)

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

# Start Crazyflie TCP client
try:
    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger('websockets').setLevel(logging.WARNING)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(f"tcp://{HOST_IP}:8000", cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Connected!")
        scf.cf.platform.send_arming_request(True)
        print("Armed!")

        # Only open display if show_camera is True
        if args.show_camera:
            cv2.namedWindow("WebSocket Stream", cv2.WINDOW_NORMAL)

        time.sleep(1)

        with MotionCommander(scf, default_height=1) as mc:
            # Main loop
            while not shutdown_event.is_set():
                timestamp = process_frame()

            time.sleep(1)
            mc.stop()

except KeyboardInterrupt:
    print("Program interrupted")
except Exception as e:
    print(f"Error in main thread: {e}")
finally:
    # Signal the camera thread to stop
    # if cam_client:
    #     cam_client.shutdown_event.set()
    
    # Wait a moment for threads to clean up
    print("Waiting for threads to stop...")
    time.sleep(1)
    
    print("Program exited")