import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

HOST = '10.8.216.77'
PORT = 5000

uri = f"tcp://{HOST}:{PORT}"

logging.basicConfig(level=logging.DEBUG)
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    print("Connected!")