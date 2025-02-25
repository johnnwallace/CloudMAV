from cflib.cpx.transports import SocketTransport
from cflib.cpx import CPX, CPXPacket, CPXTarget, CPXFunction

def main():
    host = '10.8.216.77'
    port = 5000

    cpx = CPX(SocketTransport(host, port))
    packet = CPXPacket(function=CPXFunction.CONSOLE, destination=CPXTarget.STM32, data=b"Hello\n\0")
    # packet.lastPacket = True
    cpx.sendPacket(packet)

if __name__ == "__main__":
    main()