#!/usr/bin/env python
"""
@package lidar.py
Auto mode for the robot. Execute pre-defined missions.
"""

import rospy, time
from control.control import ControlRobot
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

# Topic to publish control commands
pub = rospy.Publisher("/control_robot", Control, queue_size=10)

time.sleep(1) # Wait for publishers to be registered.

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

if __name__ == "__main__":
    try:
        control_robot = ControlRobot(pub)
        while not rospy.is_shutdown():
            pass
            #control_robot.stop()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")