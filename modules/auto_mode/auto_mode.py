#!/usr/bin/env python
"""
@package lidar.py
Auto mode for the robot. Execute pre-defined missions.
"""

# from threading import Thread
import rospy, time, traceback, sys, datetime
from control.control import ControlRobot
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
from monitor.monitor_auto_mode import Monitor
from agrobot.msg import Coords
from std_msgs.msg import String
from threading import Thread
from logger.mission_logger import MissionLogger

try:
    if(sys.argv[1] == "graph"):
        ENABLE_GRAPH = True
    else:
        ENABLE_GRAPH = False
except:
    ENABLE_GRAPH = False

if(ENABLE_GRAPH):
    from graph import GraphData

from mission.mission import _Mission, Missions
from geometry import Point, Line, PathCalculator

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

# Topic to publish control commands
pub = rospy.Publisher("/priority_decider", Control, queue_size=10)
pub_log = rospy.Publisher("/auto_mode_log", String, queue_size=10)

time.sleep(1) # Wait for publishers to be registered.

# Monitor class
monitor = Monitor()

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

mission_logger = MissionLogger()

current_point: Point = None
old_point: Point = None

missions = Missions()
path_calcs = PathCalculator()

control_robot = ControlRobot(pub)
Thread(target=control_robot.run).start()

current_mission = None

if(ENABLE_GRAPH):
    graph_data = GraphData()
    graph_data.clean_robot_route()

def run():
    global control_robot
    if(ENABLE_GRAPH):
        global graph_data
    index = 0
    for mission in missions.get_missions():
        pub_log.publish("{} - Executing mission: {}".format(datetime.datetime.now(), mission.name))
        mission_logger.update_mission_name(mission.name)
        time.sleep(3)
        for location in mission.get_locations():
            control_robot.begin = True
            pub_log.publish("{} - Executing location: {}".format(datetime.datetime.now(), location))

            # Log Control Variable
            no_gps_data_available = False
            left_command = False
            right_command = False
            forward_command = False

            while(current_point is None or old_point is None or current_point.is_zero() or old_point.is_zero() or current_point.equal(old_point.latitude, old_point.longitude)):
                control_robot.stop()
                if(no_gps_data_available == False):
                    no_gps_data_available = True
                    pub_log.publish("{} - No GPS data available".format(datetime.datetime.now()))

            target_point_location = Point(location.get_latitude(), location.get_longitude())
            mission_line = path_calcs.get_points_between(Line(current_point, target_point_location), 10)

            mission_logger.update_mission_line(current_point.latitude, current_point.longitude, target_point_location.latitude, target_point_location.longitude)
            mission_logger.update_mission_points(mission_line)
            if(ENABLE_GRAPH):
                graph_data.set_straight_from_mission(mission_line)
            
            while True:
                mission_logger.update_robot_location(current_point.latitude, current_point.longitude)
                if(ENABLE_GRAPH):
                    graph_data.new_position_robot((round(current_point.longitude, 5), round(current_point.latitude, 5)))

                robot_line = Line(old_point, current_point)
                idx_correction, idx_closest = path_calcs.get_closest_point(mission_line, current_point)
                correction_line = Line(current_point, mission_line[idx_correction])

                mission_logger.update_robot_direction(
                    robot_line.p1.latitude, robot_line.p1.longitude,
                    robot_line.p2.latitude, robot_line.p2.longitude
                )

                mission_logger.update_correction_line(
                    correction_line.p1.latitude, correction_line.p1.longitude,
                    correction_line.p2.latitude, correction_line.p2.longitude
                )

                if(idx_closest == len(mission_line)-1): # Reached next to the last point
                    control_robot.stop()
                    pub_log.publish("{} - Location ({:10.10f}, {:10.10f}) finished".format(datetime.datetime.now(), location.latitude, location.longitude))
                    pub_log.publish("{} - Robot End Location ({:10.10f}, {:10.10f})".format(datetime.datetime.now(), current_point.latitude, current_point.longitude))
                    pub_log.publish("{} - Mission Comparison Location ({:10.10f}, {:10.10f})".format(datetime.datetime.now(), mission_line[idx_closest].latitude, mission_line[idx_closest].longitude))
                    mission_logger.update_correction_direction("Reached Point")
                    mission_logger.update_iteration(index)
                    log_file = mission_logger.do_log()
                    pub_log.publish("{} - Mission Log Write to {})".format(datetime.datetime.now(), log_file))
                    time.sleep(5)
                    break

                action, chooser, error = robot_line.get_smaller_rotation_direction(correction_line.p1, correction_line.p2)
                
                mission_logger.update_correction_direction(action)
                mission_logger.update_iteration(index)
                mission_logger.update_forward_error(error)
                mission_logger.update_correction_chooser(chooser)
                mission_logger.do_log()
                if(ENABLE_GRAPH):
                    graph_data.set_correction_direction_legend(action)
                if(action == "clockwise"):
                    control_robot.right()
                    if(right_command == False):
                        left_command = False
                        right_command = True
                        forward_command = False
                        pub_log.publish("{} - Right".format(datetime.datetime.now()))
                elif(action == "counter_clockwise"):
                    control_robot.left()
                    if(left_command == False):
                        left_command = True
                        right_command = False
                        forward_command = False
                        pub_log.publish("{} - Left".format(datetime.datetime.now()))
                elif(action == "none"):
                    control_robot.forward()
                    if(forward_command == False):
                        left_command = False
                        right_command = False
                        forward_command = True
                        pub_log.publish("{} - Forward".format(datetime.datetime.now()))
                else:
                    pub_log.publish("{} - Deu Ruim".format(datetime.datetime.now()))
                 
                index += 1
                    
        #runtime_log.info("Mission {} finished".format(mission.name))
        pub_log.publish("{} - Mission {} finished".format(datetime.datetime.now(), mission.name))
        control_robot.stop()
    pub_log.publish("{} - Finished all missions".format(datetime.datetime.now()))
    control_robot.stop()

def callback_gps(data:Coords):
    """
    Update the current and old point.
    """
    global current_point, old_point
    round_to = 6
    if(current_point is None or old_point is None):
        current_point = Point(round(data.latitude, round_to), round(data.longitude, round_to))
        old_point = Point(round(data.latitude, round_to), round(data.longitude, round_to))
    else:
        if(not current_point.equal(round(data.latitude, round_to), round(data.longitude, round_to))):
            old_point.set_point(current_point.get_latitude(), current_point.get_longitude())
            current_point.set_point(round(data.latitude, round_to), round(data.longitude, round_to))

def callback_start_mission(data: String):
    """
    Start the current mission.
    """
    global current_mission, missions
    try:
        missions = Missions()
        missions.load_mission_file()
        current_mission = Thread(target=run)
        current_mission.start()
    except:
        log.error(traceback.format_exc())
        runtime_log.error("Could not start mission.")


def callback_stop_mission(data:String):
    """
    Stop the current mission.
    """
    global control_robot
    control_robot.begin = False
    pub_log.publish("{} - Mission Stopped By stop_mission callback".format(datetime.datetime.now()))

if __name__ == "__main__":
    try:
        rospy.Subscriber("/gps", Coords, callback_gps)
        rospy.Subscriber("/start_mission", String, callback_start_mission)
        rospy.Subscriber("/stop_mission", String, callback_stop_mission)
        rospy.spin()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")