#!/usr/bin/env python
"""
@package lidar.py
Auto mode for the robot. Execute pre-defined missions.
"""

# from threading import Thread
import rospy, time, traceback
from control.control import ControlRobot
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
from monitor.monitor_auto_mode import Monitor
from agrobot.msg import Coords
from std_msgs.msg import String
from threading import Thread
from graph import GraphData
from logger.mission_logger import MissionLogger


from mission.mission import _Mission, Missions
from geometry import Point, Line, PathCalculator

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

mission_logger = MissionLogger()

current_point: Point = None
old_point: Point = None

missions = Missions()
path_calcs = PathCalculator()

control_robot = ControlRobot(pub)
Thread(target=control_robot.run).start()

current_mission = None

graph_data = GraphData()
graph_data.clean_robot_route()

def run():
    global control_robot , graph_data
    for mission in missions.get_missions():
        print(len(missions.get_missions()))
        log.info("Executing mission: {}".format(mission.name))
        for location in mission.get_locations():
            control_robot.begin = True
            runtime_log.info("Executing location: {}".format(location))

            while(current_point is None or old_point is None or current_point.is_zero() or old_point.is_zero() or current_point.equal(old_point.latitude, old_point.longitude)):
                control_robot.stop()
                runtime_log.info("No GPS data available")

            target_point_location = Point(location.get_latitude(), location.get_longitude())
            mission_line = path_calcs.get_points_between(Line(current_point, target_point_location), 10)

            mission_logger.update_mission_line(current_point.latitude, current_point.longitude, target_point_location.latitude, target_point_location.longitude)
            mission_logger.update_mission_points(mission_line)
            graph_data.set_straight_from_mission(mission_line)
            
            while True:
                mission_logger.update_robot_location(current_point.latitude, current_point.longitude)
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
                    runtime_log.info("Location ({:10.10f}, {:10.10f}) finished".format(location.latitude, location.longitude))
                    runtime_log.info("Robot End Location ({:10.10f}, {:10.10f})".format(current_point.latitude, current_point.longitude))
                    runtime_log.info("Mission comparation Location ({:10.10f}, {:10.10f})".format(mission_line[idx_closest].latitude, mission_line[idx_closest].longitude))
                    mission_logger.update_correction_direction("Reached Point")
                    mission_logger.do_log()
                    time.sleep(5)
                    break

                action = robot_line.get_smaller_rotation_direction(correction_line.p1, correction_line.p2)
                
                mission_logger.update_correction_direction(action)
                mission_logger.do_log()
                graph_data.set_correction_direction_legend(action)
                if(action == "clockwise"):
                    control_robot.right()
                    print("Right")
                elif(action == "counter_clockwise"):
                    control_robot.left()
                    print("Left")
                elif(action == "none"):
                    control_robot.forward()
                    print("Forward")
                else:
                    print("Deu Ruim")
                #input("Press Enter to continue...")
                    
        runtime_log.info("Mission {} finished".format(mission.name))
        control_robot.stop()
    runtime_log.info("Finished all missions")
    control_robot.stop()

def callback_gps(data:Coords):
    """
    Update the current and old point.
    """
    global current_point, old_point , graph_data
    round_to = 7
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
    #global current_mission
    #if(current_mission is not None):
    #    runtime_log.info("Canceled mission with callback stop mission.")
    #    current_mission.join()
    #    current_mission = None
    #else:
    #    runtime_log.warning("Could not canceled mission with callback stop mission.")


if __name__ == "__main__":
    try:
        rospy.Subscriber("/gps", Coords, callback_gps)
        rospy.Subscriber("/start_mission", String, callback_start_mission)
        rospy.Subscriber("/stop_mission", String, callback_stop_mission)
        rospy.spin()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")