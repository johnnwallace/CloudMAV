import logging
import struct
import sys
import threading
from time import time

import numpy as np

import cflib.crtp
from cflib.cpx import CPXFunction
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# def disconnected(self, URI):
#     print('Disconnected')
#     sys.exit(1)

# def connected(self, URI):
#     print('We are now connected to {}'.format(URI))

#     # The definition of the logconfig can be made before connecting
#     lp = LogConfig(name='Position', period_in_ms=100)
#     lp.add_variable('stateEstimate.x')
#     lp.add_variable('stateEstimate.y')
#     lp.add_variable('stateEstimate.z')
#     lp.add_variable('stabilizer.roll')
#     lp.add_variable('stabilizer.pitch')
#     lp.add_variable('stabilizer.yaw')

#     try:
#         self.cf.log.add_config(lp)
#         lp.data_received_cb.add_callback(self.pos_data)
#         lp.start()
#     except KeyError as e:
#         print('Could not start log configuration,'
#                 '{} not found in TOC'.format(str(e)))
#     except AttributeError:
#         print('Could not add Position log config, bad configuration.')

#     def pos_data(self, timestamp, data, logconf):
#         for name in data:
#             self.labels[name]['widget'].setText('{:.02f}'.format(data[name]))

#     def closeEvent(self, event):
#         if (self.cf is not None):
#             self.cf.close_link()

def log_config():
    out = LogConfig(name='Stabilizer', period_in_ms=10)
    out.add_variable('stabilizer.roll', 'float')
    out.add_variable('stabilizer.pitch', 'float')
    out.add_variable('stabilizer.yaw', 'float')
    return out

logging.basicConfig(level=logging.INFO)

host = '10.8.216.77'
port = 5000
URI = f"tcp://{host}:{port}"

cflib.crtp.init_drivers()

lg_stab = log_config()

cf = Crazyflie(ro_cache=None, rw_cache='cache')

# Connect callbacks from the Crazyflie API
# cf.connected.add_callback(connected)
# cf.disconnected.add_callback(disconnected)

# Connect to the Crazyflie
cf.open_link(URI)

if not cf.link:
    print('Could not connect to Crazyflie')
    sys.exit(1)

if not hasattr(cf.link, 'cpx'):
    print('Not connecting with WiFi')
    cf.close_link()
else:
    print('Connected to Crazyflie')

    

    with SyncCrazyflie(URI, cf=cf) as scf:
        with SyncLogger(scf, lg_stab) as logger:
            endTime = time.time() + 10

            for log_entry in logger:
                timestamp = log_entry[0]
                data = log_entry[1]
                logconf_name = log_entry[2]

                print('[%d][%s]: %s' % (timestamp, logconf_name, data))

                if time.time() > endTime:
                    break
    # self._imgDownload = ImageDownloader(self.cf.link.cpx, self.updateImage)
    # self._imgDownload.start()

    # self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

    # # Arm the Crazyflie
    # self.cf.platform.send_arming_request(True)
    # time.sleep(1.0)

    # self.hoverTimer = QtCore.QTimer()
    # self.hoverTimer.timeout.connect(self.sendHoverCommand)
    # self.hoverTimer.setInterval(100)
    # self.hoverTimer.start()