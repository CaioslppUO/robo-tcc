#!/usr/bin/env python
"""
@package mission.py
Control the automatic mode.
"""

import rospy, time
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from mission import Missions, _Location
from agrobot.msg import Coords, Control
import auto_mode_calcs

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

pub = rospy.Publisher("/get_robot_commands", Control, queue_size=10)

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

# Global control variables
stop_mission: bool = False
lidar_can_move: bool = True
robot_latitude: float = 0.0
robot_longitude: float = 0.0
robot_compass: float = 0.0

def verify_coordinates(location: _Location) -> bool:
    """
    Verify if the robot is in the correct position.
    """
    decimals = 4
    if (round(location.latitude, decimals) != round(robot_latitude, decimals) or round(location.longitude, decimals) != round(robot_longitude, decimals)):
        return True
    return False

def send_control_command(speed: float, steer: float, timeout: float) -> None:
    control: Control = Control()
    control.speed = speed
    control.steer = steer
    pub.publish(control)
    time.sleep(timeout)

def turn_left() -> None:
    turn_left_timeout = 0.5
    runtime_log.info("Turn Left")
    send_control_command(0.0, -0.5, turn_left_timeout)
    send_control_command(0.5, 0.0, turn_left_timeout)
    send_control_command(0.0, 0.5, turn_left_timeout)
    send_control_command(0.5, 0.0, turn_left_timeout)
    send_control_command(0.0, 0.0, 0.0)

def turn_right() -> None:
    turn_right_timeout = 0.5
    runtime_log.info("Turn Right")
    send_control_command(0.0, 0.5, turn_right_timeout)
    send_control_command(0.5, 0.0, turn_right_timeout)
    send_control_command(0.0, -0.5, turn_right_timeout)
    send_control_command(0.5, 0.0, turn_right_timeout)
    send_control_command(0.0, 0.0, 0.0)

def go_forward() -> None:
    go_forward_timeout = 0.0
    runtime_log.info("Go Forward")
    send_control_command(0.5, 0.0, go_forward_timeout)

def run(missions: Missions) -> None:
    """
    Execute the mission.
    """
    direct_of_correct: str = need_to_correct_route(location)
    control: Control = Control()
    control.speed = 1
    while (not stop_mission):
        for mission in missions.get_missions():  # Execute every mission
            if (stop_mission): return

            for location in mission.get_locations():  # Execute every location
                if (stop_mission): return

                while verify_coordinates(location):  # Verifica se chegou ao destino da localização.
                    if (stop_mission): return

                    if (lidar_can_move):
                        while direct_of_correct != "left" and direct_of_correct != "right":  # Lógica para corrigir a direção do robô
                            if (stop_mission): return None
                            
                            direct_of_correct = need_to_correct_route(location)

                            if direct_of_correct == "left":
                                turn_left()
                            elif direct_of_correct == "right":
                                turn_right()

                        go_forward()
                    else:
                        runtime_log.info("Can't move, object in front of robot (lidar).")


def need_to_correct_route(location: _Location) -> str:
    """
    Verify if the robot needs to correct the route.
    """
    global robot_compass

    # Setting robot in (0,0) coordinate in the system
    new_robot_latitude: float = 0.0
    new_robot_longitude: float = 0.0

    # Setting the mission lat and lon to be relative to robot (0,0)
    new_latitude: float = location.latitude - robot_latitude
    new_longitude: float = location.longitude - robot_longitude

    # Error trying to read robot lat and lon
    if robot_latitude >= 0 or robot_longitude >= 0:
        runtime_log.warning("Robot cold not read GPS coordinates.")
        return "error"

    H = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_latitude, new_longitude)
    co = auto_mode_calcs.dist_two_points(new_robot_latitude, new_robot_longitude, new_robot_latitude, new_longitude)
    Beta = 90 - auto_mode_calcs.get_angle_asin(co, H)

    if new_longitude < 0:  # Quadrante baixo
        if new_latitude < 0:  # Quadrante esquerda
            Beta = Beta + 180
            if robot_compass >= Beta or robot_compass <= Beta-180:
                return "left" #Virar esquerda
            return "right" #Virar direita
        else:  # Quadrante direita
            Beta = 180 - Beta
            if robot_compass <= Beta or robot_compass >=  Beta+180:
                return "right" #Virar Direita
            return "left" #Virar esquerda
    else:  # Quadrante cima
        if new_latitude < 0:  # Quadrante esquerda
            if 360-Beta >= robot_compass or robot_compass <= 180-Beta:
                return "left" #Virar Esquerda
            return "right" #Virar Direita
        else:  # Quadrante direita
            if robot_compass >= 360-Beta or robot_compass <= Beta:
                return "right" #Virar Direita
            return "left" #Virar Esquerda

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