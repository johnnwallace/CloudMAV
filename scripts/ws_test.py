import asyncio
import websockets

async def test_websocket():
    uri = "ws://10.8.186.141:80/ws"
    async with websockets.connect(uri, ping_interval=None) as websocket:
        await websocket.send("Hello")
        response = await websocket.recv()
        print(f"Received: {response}")

asyncio.run(test_websocket())

# import socket

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect(("10.8.186.141", 80))
# s.send(b"GET /ws HTTP/1.1\r\nHost: 10.8.186.141\r\n\r\n")
# data = s.recv(1024)
# print(data)
# s.close()