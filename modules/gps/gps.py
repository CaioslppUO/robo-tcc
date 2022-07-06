#!/usr/bin/env python
"""
@package gps.py
Read gps data from serial port and publish to topic
"""

from ublox_gps import UbloxGps
from serial import Serial

import rospy
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.param import Parameter
from agrobot.msg import Coords
import traceback

# Lidar node
rospy.init_node('gps', anonymous=True)

# Publication topic of this node.
pub = rospy.Publisher("/gps", Coords, queue_size=10)

# Log class
log: Log = Log("gps.py")
runtime_log: RuntimeLog = RuntimeLog("gps.py")

# Global variables
priorities: dict = {}

# Parameter class
param: Parameter = Parameter()

def get_rosparam_priorities() -> None:
    """
    Get priorities from rosparam.
    """
    global priorities
    priorities.update({"USB_PORT_GPS": param.get_param("USB_PORT_GPS")})


def run(gps):
    '''
    Read the ublox gps data and publish in to topic.
    '''
    try:
        cds: Coords = Coords()
        coords = gps.geo_coords()
        cds.latitude = coords.lat
        cds.longitude = coords.lon
        pub.publish(cds)
    except:
        runtime_log.error("read gps failed.")
        log.error(traceback.format_exc())


if __name__ == '__main__':
    try:
        get_rosparam_priorities()
        port = Serial(priorities["USB_PORT_GPS"], baudrate=9600, timeout=1)
        gps = UbloxGps(port)
        while not rospy.is_shutdown():
            run(gps)
    except:
        log.error(traceback.format_exc())
        runtime_log.error("gps.py terminated")

