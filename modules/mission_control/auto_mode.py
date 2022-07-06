#!/usr/bin/env python
"""
@package mission.py
Control the automatic mode.
"""

from sqlalchemy import Float, true
import rospy
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from mission import Missions, _Location
from agrobot.msg import Coords

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

# Global control variables
stop_mission = False
lidar_can_move: bool = True
robot_latitude: float = 0.0
robot_longitude: float = 0.0
robot_compass = None


def run(missions: Missions) -> None:
    """
    Execute the mission.
    """
    while (not stop_mission):
        for mission in missions.get_missions():  # Execute every mission
            if (stop_mission):
                return

            for location in mission.get_locations():  # Execute every location
                if (stop_mission):
                    return

                while verify_coordinates(
                        location
                ):  # Verifica se chegou ao destino da localização.
                    if (stop_mission):
                        return None

                    if (lidar_can_move):
                        while need_to_correct_route(
                                location
                        ):  # Lógica para corrigir a direção do robô
                            if (stop_mission):
                                return None

                        # Vai reto
                    else:
                        runtime_log.info(
                            "Can't move, object in front of robot (lidar).")


def need_to_correct_route(location: _Location) -> bool:
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
        return False
    if new_longitude < 0:  # Quadrante baixo
        if new_longitude < 0:  # Quadrante esquerda

            return True
        else:  # Quadrante direita
            return False
    else:  # Quadrante cima
        if new_longitude < 0:  # Quadrante esquerda
            return True
        else:  # Quadrante direita
            return False


def verify_coordinates(location: _Location) -> bool:
    """
    Verify if the robot is in the correct position.
    """
    if (location.latitude != robot_latitude or location.longitude != robot_longitude):
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