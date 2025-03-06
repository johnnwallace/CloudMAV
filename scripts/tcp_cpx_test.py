import time
import socket

from cflib.cpx.transports import SocketTransport
from cflib.cpx import CPX, CPXPacket, CPXTarget, CPXFunction

def main():
    host = '10.9.146.64'
    # host = '192.168.1.168'
    port = 80

    cpx = CPX(SocketTransport(host, port))
    packet = CPXPacket(function=CPXFunction.CONSOLE, destination=CPXTarget.STM32, data=b"Hello\n\0")
    # packet.lastPacket = True

    while True:
        try:
            cpx.sendPacket(packet)
            time.sleep(2)
            print("Sending...")
        except KeyboardInterrupt:
            break

    cpx.close()

if __name__ == "__main__":
    main()