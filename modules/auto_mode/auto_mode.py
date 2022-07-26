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


from mission.mission import _Mission,Missions
from geometry import Point, Line

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
control_robot = ControlRobot(pub)
Thread(target=control_robot.run).start()

current_mission = None

# graph_data = GraphData()

def get_points_between(line_target: Line, number_of_points: int) -> "list[Point]":
    """
    Return number_of_points between start and end.
    """
    points: "list[Point]" = []
    longitude = line_target.p1.longitude
    for i in range(0, number_of_points - 1):
        latitude = round(line_target.angular_coefficient * longitude + line_target.linear_coefficient, 7)
        points.append(Point(latitude, longitude))
        if(line_target.p2.longitude > line_target.p1.longitude):
            longitude += line_target.p1.difference(line_target.p2.latitude, line_target.p2.longitude)[1] / (number_of_points - 1)
        else:
            longitude -= line_target.p1.difference(line_target.p2.latitude, line_target.p2.longitude)[1] / (number_of_points - 1)
    points.append(line_target.p2)
    return points

def get_closest_point(line_target:"list[Point]", robot:Point) -> int:
    """
    Return the index of the closest point to the robot.
    """
    distances:"list[float]" = []
    min_dist = -1
    index = 0

    for point in line_target:
        distances.append(point.distance(robot.latitude, robot.longitude))
    for i in range(len(distances)):
        if min_dist == -1 or distances[i] < min_dist:
            min_dist = distances[i]
            index = i
    correction_point = index + 5
    if(correction_point >= len(distances)):
            correction_point = len(distances)-1

    return correction_point


def run():
    global control_robot#, graph_data
    for mission in missions.get_missions():
        log.info("Executing mission: {}".format(mission.name))
        for location in mission.get_locations():
            runtime_log.info("Executing location: {}".format(location))
            if(current_point is None or old_point is None or current_point.is_zero() or old_point.is_zero()):
                control_robot.stop()
                runtime_log.info("No GPS data available")
                continue

            target_point_location = Point(location.get_longitude(), location.get_latitude())
            mission_line = get_points_between(Line(current_point, target_point_location),10)

            mission_logger.update_mission_line(current_point.latitude, current_point.longitude, target_point_location.latitude, target_point_location.longitude)
            mission_logger.update_mission_points(mission_line)
            # graph_data.set_straight_from_mission(mission_line)
            
            while True:
                mission_logger.update_robot_location(current_point.latitude, current_point.longitude)
                # graph_data.new_position_robot((round(current_point.longitude, 5), round(current_point.latitude, 5)))

                robot_line = Line(old_point, current_point)
                idx = get_closest_point(mission_line, current_point)
                correction_line = Line(current_point, mission_line[idx])

                mission_logger.update_robot_direction(
                    robot_line.p1.latitude, robot_line.p1.longitude,
                    robot_line.p2.latitude, robot_line.p2.longitude
                )

                mission_logger.update_correction_line(
                    correction_line.p1.latitude, correction_line.p1.longitude,
                    correction_line.p2.latitude, correction_line.p2.longitude
                )

                if(idx == len(mission_line)-1): # Reached next to the last point
                    control_robot.stop()
                    runtime_log.info("Location ({:10.10f}, {:10.10f}) finished".format(location.latitude, location.longitude))
                    mission_logger.update_correction_direction("Reached Point")
                    mission_logger.do_log()
                    time.sleep(5)
                    break

                action = robot_line.get_smaller_rotation_direction(correction_line.p1, correction_line.p2)
                
                mission_logger.update_correction_direction(action)
                mission_logger.do_log()
                # graph_data.set_correction_direction_legend(action)
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
    global current_point, old_point#, graph_data
    if(current_point is None or old_point is None):
        current_point = Point(round(data.latitude, 5), round(data.longitude, 5))
        old_point = Point(round(data.latitude, 5), round(data.longitude, 5))
    else:
        if(not current_point.equal(round(data.latitude, 5), round(data.longitude, 5))):
            old_point.set_point(current_point.get_latitude(), current_point.get_longitude())
            current_point.set_point(round(data.latitude, 5), round(data.longitude, 5))

def callback_start_mission(data:String):
    """
    Start the current mission.
    """
    global current_mission,missions
    try:
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
    global current_mission
    if(current_mission is not None):
        runtime_log.info("Canceled mission with callback stop mission.")
        current_mission.join()
        current_mission = None
    else:
        runtime_log.warning("Could not canceled mission with callback stop mission.")


if __name__ == "__main__":
    try:
        rospy.Subscriber("/gps", Coords, callback_gps)
        rospy.Subscriber("/start_mission", String, callback_start_mission)
        rospy.Subscriber("/stop_mission", String, callback_stop_mission)
        rospy.spin()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")