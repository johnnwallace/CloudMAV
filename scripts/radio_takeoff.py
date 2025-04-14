import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

uri = 'radio://0/1/2M/E7E7E7E7E7'

logging.basicConfig(level=logging.DEBUG)
# logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    # turn off link health check

    print("Connected!")
    scf.cf.platform.send_arming_request(True)
    print("Amrmed!")
    with MotionCommander(scf, default_height=1) as mc:
        time.sleep(1)
        mc.stop()