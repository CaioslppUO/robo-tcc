#!/usr/bin/env python
"""
@package mission.py
Control the automatic mode.
"""

import rospy
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from mission import Missions, _Location
from agrobot.msg import Coords,Control
import auto_mode_calcs



# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

pub = rospy.Publisher("/auto_mode", Control, queue_size=10)

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

# Global control variables
stop_mission: bool = False
lidar_can_move: bool = True
robot_latitude: float = 0.0
robot_longitude: float = 0.0
robot_compass: float = 0.0


def run(missions: Missions) -> None:
    """
    Execute the mission.
    """
    direct_of_correct:int = 0
    control: Control = Control()
    control.speed = 1
    while (not stop_mission):
        for mission in missions.get_missions():  # Execute every mission
            if (stop_mission): return
            for location in mission.get_locations():  # Execute every location
                if (stop_mission): return
                while verify_coordinates(location):  # Verifica se chegou ao destino da localização.
                    if (stop_mission): return None
                    if (lidar_can_move):
                        while direct_of_correct != 2:  # Lógica para corrigir a direção do robô
                            direct_of_correct = need_to_correct_route(location)
                            if direct_of_correct == 0:
                                runtime_log.info("Virar esquerda")
                                control.speed = 0.0
                                control.steer = -0.5
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.5
                                control.steer = 0
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.0
                                control.steer = 0.5
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.5
                                control.steer = 0
                                pub.publish(control)
                                rospy.sleep(1)
                            elif direct_of_correct == 1:
                                runtime_log.info("Virar direita")
                                control.speed = 0.0
                                control.steer = 0.5
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.5
                                control.steer = 0
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.0
                                control.steer = -0.5
                                pub.publish(control)
                                rospy.sleep(1)
                                control.speed = 0.5
                                control.steer = 0
                                pub.publish(control)
                                rospy.sleep(1)
                            if (stop_mission): return None
                        control.speed = 0.8
                        control.steer = 0
                        pub.publish(control)
                    else:
                        runtime_log.info("Can't move, object in front of robot (lidar).")


def need_to_correct_route(location: _Location) -> int:
    global robot_compass
    """
    Verify if the robot needs to correct the route.
    """

    new_robot_latitude: float = 0.0
    new_robot_longitude: float = 0.0
    new_latitude: float = location.latitude - robot_latitude
    new_longitude: float = location.longitude - robot_longitude

    if robot_latitude == 0 and robot_longitude == 0:
        runtime_log.warning("Robot cold not get GPS coordinates.")
        return 2
    if new_longitude < 0:  # Quadrante baixo
        if new_longitude < 0:  # Quadrante esquerda
            H = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_latitude, new_longitude)
            co = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_robot_latitude, new_longitude)
            Beta = 90- auto_mode_calcs.get_angle_asin(co,H)
            Beta = Beta+180
            if robot_compass >= Beta or robot_compass <= Beta-180:
                return 0 #Virar esquerda
            return 1 #Virar direita
        else:  # Quadrante direita
            H = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_latitude, new_longitude)
            co = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_robot_latitude, new_longitude)
            Beta = 90- auto_mode_calcs.get_angle_asin(co,H)
            Beta = 180 - Beta
            if robot_compass <= Beta or robot_compass >=  Beta+180:
                return 1 #Virar Direita
            return 0 #Virar esquerda
    else:  # Quadrante cima
        if new_longitude < 0:  # Quadrante esquerda
            H = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_latitude, new_longitude)
            co = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_robot_latitude, new_longitude)
            Beta = 90- auto_mode_calcs.get_angle_asin(co,H)
            if 360-Beta >= robot_compass or robot_compass <= 180-Beta:
                return 0 #Virar Esquerda
            return 1 #Virar Direita
        else:  # Quadrante direita
            H = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_latitude, new_longitude)
            co = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_robot_latitude, new_longitude)
            Beta = 90- auto_mode_calcs.get_angle_asin(co,H)
            if robot_compass >= 360-Beta or robot_compass <= Beta:
                return 1 #Virar Direita
            return 0 #Virar Esquerda
    return 2

def verify_coordinates(location: _Location) -> bool:
    """
    Verify if the robot is in the correct position.
    """
    if (round(location.latitude,4) != round(robot_latitude,4) or round(location.longitude,4) != round(robot_longitude,4)):
        return True
    return False


def callback_lidar(dt: String):
    """
    Update the lidar object detection value.
    """
    global lidar_can_move
    lidar_can_move = bool(dt.data)


def callback_start_mission(dt: String):
    """
    Load and execute a new mission file.
    """
    missions = Missions()
    missions.load_mission_file()
    run(missions)


def callback_stop_mission(dt: String):
    """
    Stop an mission that is being executed.
    """
    global stop_mission
    stop_mission = False


def callback_gps(dt: Coords) -> None:
    """
    Update current robot latitude and longitude
    """
    global robot_latitude, robot_longitude
    robot_latitude = float(dt.latitude)
    robot_longitude = float(dt.longitude)


def callback_compass(dt: String) -> None:
    """
    Update current robot latitude and longitude
    """
    global robot_compass
    robot_compass = float(dt.data)


if __name__ == "__main__":
    try:
        rospy.Subscriber('/lidar', String, callback_lidar)
        rospy.Subscriber('/gps', Coords, callback_gps)
        rospy.Subscriber('/compass', String, callback_compass)
        rospy.Subscriber('/start_mission', String, callback_start_mission)
        rospy.Subscriber('/stop_mission', String, callback_stop_mission)
        rospy.spin()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")