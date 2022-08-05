#!/usr/bin/env python

"""
@package gps_simulation.py
Emulate a GPS by publishing to ROS real data extracted from a mission log file of the robot.
"""

import traceback, time, sys
from mission_data_analyzer import MissionDataAnalyzer

try:
    from agrobot.msg import Coords
    import rospy
    from std_msgs.msg import String
    rospy.init_node("gps_simulation", anonymous=True)
    pub = rospy.Publisher("/gps", Coords, queue_size=10)
    pub_stop = rospy.Publisher("/stop_mission", String, queue_size=10)
    pub_start = rospy.Publisher("/start_mission", String, queue_size=10)
    time.sleep(1)
except:
    print(traceback.format_exc())

class GPSSimulation:
    def __init__(self, mission_logs_file: str) -> None:
         self.mission_analyzer = MissionDataAnalyzer(mission_logs_file)

    def publish_fake_gps_data(self) -> None:
        """
        Read a mission log file, extract robot_gps positions and publish to the ROS, simulating the real robot.
        """
        gps_points = self.mission_analyzer.get_gps_robot_positions()
        pub_start.publish("start")
        time.sleep(2)
        for g in gps_points:
            c = Coords()
            c.latitude = g[0]
            c.longitude = g[1]
            pub.publish(c)
            time.sleep(0.1)
        pub_stop.publish("stop")

if __name__ == "__main__":
    try:
        file = sys.argv[1]
        gp = GPSSimulation(file)
        gp.publish_fake_gps_data()
    except:
        print(traceback.format_exc())