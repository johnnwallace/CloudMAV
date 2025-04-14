import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander


# HOST = '10.9.146.64'
HOST = '10.8.186.141'

uri = f"wss://{HOST}/ws"

logging.basicConfig(level=logging.DEBUG)
# logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    # turn off link health check

    print("Connected!")
    scf.cf.platform.send_arming_request(True)
    print("Amrmed!")
    with MotionCommander(scf, default_height=0.5) as mc:
        time.sleep(1)
        mc.stop()