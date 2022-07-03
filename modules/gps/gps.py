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
from agrobot.msg import Coords
import traceback

# Lidar node
rospy.init_node('gps', anonymous=True)

# Publication topic of this node.
pub = rospy.Publisher("/lat_lng", Coords, queue_size=10)

# Log class
log: Log = Log("gps.py")
runtime_log: RuntimeLog = RuntimeLog("gps.py")


def run(gps):
    '''
    Read the ublox gps data and publish in to topic.
    '''
    try:
        cds: Coords = Coords()
        coords = gps.geo_coords()
        cds.latitude = coords.latitude
        cds.longitude = coords.longitude
        pub.publisher(cds)
    except (ValueError, IOError) as err:
        log.error(err)


if __name__ == '__main__':
    try:
        port = Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        gps = UbloxGps(port)
        while not rospy.is_shutdown():
            run(gps)
        port.close()
    except:
        log.error(traceback.format_exc())
