from websockets.sync.client import connect


def hello():
    with connect("ws://10.8.186.141:80/ws") as websocket:
        websocket.send("Hello world!")
        message = websocket.recv()
        print(message)


if __name__ == "__main__":
    hello()