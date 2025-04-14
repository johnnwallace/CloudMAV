import ssl
import asyncio
import cv2
import numpy as np
import websockets
from io import BytesIO
from PIL import Image

async def receive_and_display_images(websocket_url):
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.load_verify_locations("cloud-deck/certificates/cacert.pem")
    ssl_context.check_hostname = False
    
    cv2.namedWindow("WebSocket Stream", cv2.WINDOW_NORMAL)
    
    async with websockets.connect(websocket_url, ssl=ssl_context) as websocket:
        print(f"Connected to {websocket_url}")
        
        while True:
            try:
                # Add timeout to detect stalled connections
                image_bytes = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                
                # Convert bytes to numpy array using PIL as intermediary
                try:
                    pil_image = Image.open(BytesIO(image_bytes))
                    frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
                    
                    # Display with OpenCV (much faster than matplotlib)
                    cv2.imshow("WebSocket Stream", frame)
                    
                    # Process events with small timeout
                    key = cv2.waitKey(1)
                    if key == 27:  # ESC key
                        break
                        
                    print(f"Frame displayed: {frame.shape[1]}x{frame.shape[0]}")
                    
                    # Short delay to allow server to catch up if needed
                    await asyncio.sleep(0.01)
                    
                except Exception as e:
                    print(f"Image decoding error: {e}")
                    
            except asyncio.TimeoutError:
                print("Timeout waiting for frame")
                break
            except Exception as e:
                print(f"Connection error: {e}")
                break
    
    cv2.destroyAllWindows()

# Main function - now with proper shutdown handling
async def main():
    websocket_url = "wss://192.168.1.168/ws"
    try:
        await receive_and_display_images(websocket_url)
    except KeyboardInterrupt:
        print("Shutting down gracefully")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())