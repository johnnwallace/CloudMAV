import time

from cflib.cpx.transports import SocketTransport
from cflib.cpx import CPX, CPXPacket, CPXTarget, CPXFunction
from cflib.crtp import TcpDriver
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

host = '10.8.216.77'
port = 5000
URI = f"tcp://{host}:{port}"

driver = TcpDriver()
driver.connect(URI, None, None)

packet = CRTPPacket()
packet.set_header(CRTPPort.LINKCTRL, 3)
# packet.data = b"Hello\n\0"

# while True:
driver.send_packet(packet)
print("Sending packet...")
time.sleep(1)
driver.send_packet(packet)
print("Sending packet...")
time.sleep(1)
driver.send_packet(packet)
print("Sending packet...")

# driver.close()

# cpx = CPX(SocketTransport(host, port))
# packet = CPXPacket(function=CPXFunction.CONSOLE, destination=CPXTarget.STM32, data=b"Hello\n\0")
# # packet.lastPacket = True
# cpx.sendPacket(packet)
