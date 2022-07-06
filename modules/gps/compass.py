#!/usr/bin/env python
"""
@package gps.py
Read compass data from MPU950 and publish to topic
Ref: https://codigos.ufsc.br/lapix/intelligent-vehicle-perception-based-on-inertial-sensing/-/tree/master/MPU-9250%20Sensors%20Data%20Collect
"""

import time, math
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *
import rospy
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from std_msgs.msg import String
import traceback

# Log class
log: Log = Log("compass.py")
runtime_log: RuntimeLog = RuntimeLog("compass.py")
COULD_SETUP_MPU = True

# Encoder node
rospy.init_node('compass', anonymous=True)

# Publication topic
pub: rospy.Publisher = rospy.Publisher("/compass", String, queue_size=10)

class MPU:
    def __init__(self) -> None:
        try:
            self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=MPU9050_ADDRESS_68,
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_1000,
                afs=AFS_8G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ
            )
            self.mpu.configure()
        except:
            global COULD_SETUP_MPU
            COULD_SETUP_MPU = False
            log.warning(traceback.format_exc())
            runtime_log.warning("Could not setup MPU Compass Sensor")
    
    def get_angle(self) -> int:
        try:
            x, y, z = self.mpu.readMagnetometerMaster()
            heading = math.atan2(x, y)
            heading = int(heading * 180 / math.pi)
            if(heading < 0):
                heading = abs(heading+360)
            return heading
        except:
            log.warning(traceback.format_exc())
            if(COULD_SETUP_MPU):
                runtime_log.warning("Could not read angle from MPU Compass Sensor.")

def publish_compass(angle: int) -> None:
    """
    Publish compass data to ROS /compass topic.
    """
    try:
        pub.publish(str(angle))
    except:
        log.warning(traceback.format_exc())
        if(COULD_SETUP_MPU):
            runtime_log.warning("Could not publish compass data to /compass.")

if __name__ == "__main__":
    try:
        mpu = MPU()
        while not rospy.is_shutdown():
            angle = mpu.get_angle() # Read data MPU
            publish_compass(angle) # Publish compass data to ROS
            time.sleep(1) # Wait before get next data
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Compass finished. Check logs.")