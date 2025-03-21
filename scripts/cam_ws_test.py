import asyncio
import io
import matplotlib.pyplot as plt
import websockets
from PIL import Image

# Create a figure and axis for displaying the image
fig, ax = plt.subplots()
image_display = None

async def receive_and_display_images(websocket_url):
    global image_display
    
    async with websockets.connect(websocket_url) as websocket:
        print(f"Connected to {websocket_url}")
        
        # Keep the display window open and interactive
        plt.ion()
        plt.show()
        
        while True:
            try:
                # Receive binary data from the websocket
                image_bytes = await websocket.recv()
                
                # Convert the bytes to an image using PIL
                image = Image.open(io.BytesIO(image_bytes))
                
                # Display or update the image
                if image_display is None:
                    # First image: create the display
                    image_display = ax.imshow(image)
                    ax.set_title("Websocket Image Stream")
                    ax.axis('off')  # Hide the axes
                else:
                    # Update the existing display with new image data
                    image_display.set_data(image)
                
                # Adjust the figure if image dimensions change
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                print("Image updated")
                
            except Exception as e:
                print(f"Error: {e}")
                break

# Example usage
async def main():
    # Replace with your actual websocket URL
    websocket_url = "ws://10.8.186.141/ws"
    await receive_and_display_images(websocket_url)

if __name__ == "__main__":
    asyncio.run(main())