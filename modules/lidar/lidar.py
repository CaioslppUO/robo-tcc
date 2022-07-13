#!/usr/bin/env python
"""
@package lidar.py
Interpret lidar data and send to the control_module if the robot can or can't move.
"""

from control_movement import ControlMovement
from process_lidar import ProcessLidar
import rospy
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
import traceback
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# Lidar node
rospy.init_node('lidar', anonymous=True)

# Publication topic of this node.
pub = rospy.Publisher("/lidar", String, queue_size=10)

# Log class
log: Log = Log("lidar.py")
runtime_log: RuntimeLog = RuntimeLog("lidar.py")

# ControlMovement module
secure_distance = 0.35 # Minimum distance which the robot must keep from objects in centimeters
control_movement = ControlMovement(secure_distance)

# ProcessLidar module
process_lidar = ProcessLidar()

# Publication value control.
last_published_message = str(control_movement.can_move())

# Detection Constants
central_point = 0
number_of_points = 16

def callback_lidar_sensor(data) -> None:
    """
    Receive data from lidar sensor and update the ControleMovement module.
    If the movement permission change, send it to the /is_movement_allowed_lidar topic.
    """
    global last_published_message
    points = process_lidar.select_points(data.ranges, number_of_points, central_point)
    distance = process_lidar.get_closest_distance(points) # Shortest distance object in front of the robot.
    
    # Update the object detector module
    control_movement.object_detector.set_distance(distance)
    
    # Verify if movement permission has changed
    is_movement_allowed = str(control_movement.can_move())
    if(last_published_message != is_movement_allowed):
        pub.publish(is_movement_allowed)
        last_published_message = is_movement_allowed
if __name__ == "__main__":
    try:
        rospy.Subscriber('/scan', LaserScan, callback_lidar_sensor)
        rospy.spin()
    except:
        print(traceback.format_exc())
