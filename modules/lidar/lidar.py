"""
@package lidar.py
Interpret lidar data and send to the control_module if the robot can or can't move.
"""

from control_movement import ControlMovement
import rospy
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
import traceback
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# Lidar node
rospy.init_node('lidar', anonymous=True)

# Publication topic of this node.
pub = rospy.Publisher("/is_movement_allowed", String, queue_size=10)

# Log class
log: Log = Log("encoder.py")
runtime_log: RuntimeLog = RuntimeLog("encoder.py")

# ControlMovement module
secure_distance = 45 # Minimum distance which the robot must keep from objects
control_movement = ControlMovement(secure_distance)

# Publication value control.
last_published_message = str(control_movement.can_move())

def callback_lidar_sensor(data) -> None:
    """
    Receive data from lidar sensor and update the ControleMovement module.
    If the movement permission change, send it to the /is_movement_allowed topic.
    """
    left, front, right = 60, 60, 60 # Values from shortest distance of objects in left, front and right of the robot.
    
    # Update the object detector module
    control_movement.object_detector.set_left(left)
    control_movement.object_detector.set_left(front)
    control_movement.object_detector.set_left(right)

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