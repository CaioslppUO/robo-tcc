#!/usr/bin/env python
"""
@package lidar.py
Auto mode for the robot. Execute pre-defined missions.
"""

from threading import Thread
import rospy, time
from control.control import ControlRobot
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
from monitor.monitor_auto_mode import Monitor

# from geometry.point import Point

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

# Topic to publish control commands
pub = rospy.Publisher("/priority_decider", Control, queue_size=10)

time.sleep(1) # Wait for publishers to be registered.

# Monitor class
monitor = Monitor()

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

if __name__ == "__main__":
    try:
        control_robot = ControlRobot(pub)
        Thread(target=control_robot.run).start()
        #Thread(target=monitor.run).start()
        #control_robot.run_test()
        #while not rospy.is_shutdown():
        #    pass
        while True:
            inpt = input("ação: (fd, lt, rt, st)")
            if(inpt == "fd"):
                control_robot.forward()
            elif(inpt == "lt"):
                control_robot.left()
            elif(inpt == "rt"):
                control_robot.right()
            else:
                control_robot.stop()
    except KeyboardInterrupt:
        exit(0)
    except:
        runtime_log.error("Auto Mode died. Check logs file.")