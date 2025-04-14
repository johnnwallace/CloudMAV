import ssl
from websockets.sync.client import connect


def hello():
    # Create an SSL context that doesn't verify the certificate
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.load_verify_locations("cloud-deck/certificates/cacert.pem")
    ssl_context.check_hostname = False

    with connect("wss://10.8.186.141:443/ws", ssl_context=ssl_context) as websocket:
        websocket.send("Hello world!")
        message = websocket.recv()
        print(message)


if __name__ == "__main__":
    hello()