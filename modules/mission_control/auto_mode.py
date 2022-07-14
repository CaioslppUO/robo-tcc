#!/usr/bin/env python
"""
@package mission.py
Control the automatic mode.
"""

from modules.mission_control.point import Point
import rospy, time
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from mission import Missions, _Location
from agrobot.msg import Coords, Control
import auto_mode_calcs
from path_correction import PathCorrection

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

pub = rospy.Publisher("/control_robot", Control, queue_size=10)

# Log class
log: Log = Log("auto_mode.py")
runtime_log: RuntimeLog = RuntimeLog("auto_mode.py")

# Global control variables
stop_mission: bool = False
lidar_can_move: bool = True
robot_latitude: float = 0.0
robot_longitude: float = 0.0
robot_compass: float = 0.0

def send_control_command(speed: float, steer: float, timeout: int) -> None:
    control: Control = Control()
    control.speed = speed
    control.steer = steer
    for _ in range(0, timeout):
        pub.publish(control)

def turn_left() -> None:
    turn_left_timeout = 50
    speed = 0.2
    steer = 0.2
    send_control_command(speed, steer, turn_left_timeout)
    send_control_command(speed, 0.0, turn_left_timeout)

def turn_right() -> None:
    turn_right_timeout = 50
    speed = 0.2
    steer = 0.2
    send_control_command(speed, steer, turn_right_timeout)
    send_control_command(speed, 0.0, turn_right_timeout)

def go_forward() -> None:
    go_forward_timeout = 50
    speed = 0.2
    send_control_command(speed, 0.0, go_forward_timeout)

def verify_coordinates(location: _Location) -> bool:
    """
    Verify if the robot is in the correct position.
    """
    #print("robot: {}, {} || mission: {}, {}".format())
    error = 0.00005
    lat = robot_latitude < location.latitude - error or robot_latitude > location.latitude + error
    lon = robot_longitude < location.longitude - error or robot_longitude > location.longitude + error
    if (lat or lon):
        return False
    send_control_command(0.0, 0.0, 50)
    print("Chegou no ponto {}, {}".format(location.latitude, location.longitude)) 
    time.sleep(5)
    return True

def run(missions: Missions) -> None:
    """
    Execute the mission.
    """
    global stop_mission
    for mission in missions.get_missions():  # Execute every mission
        if (stop_mission): return
        print("Executando missao: {}".format(mission.name))

        for location in mission.get_locations():  # Execute every location
            # Defin mission constants
            correction_point_distance = 4
            number_of_points = 16
            robot_location = Point(robot_latitude, robot_longitude)
            next_location = Point(location.latitude, location.longitude)

            # Mission Path
            path = PathCorrection(robot_location, next_location, number_of_points)
            points_to_reach, n_of_points = path.get_points_between()
            closest_point, correction_point = points_to_reach.get_closest_points(
                robot_location,
                correction_point_distance
            )

            # Path to the closest point
            path_closest = PathCorrection(
                robot_location,
                closest_point,
                1
            )
            # Direction to go back to mission
            direction_of_correct: str = robot_location.get_correction_direction(
                closest_point.latitude, 
                closest_point.longitude, 
                path_closest.angular_coff
            )

            if (stop_mission): return
            print("Executando localizacao: {}, {}".format(location.latitude,location.longitude))
            print("Correcao direcao: {}".format(direction_of_correct))

            while not verify_coordinates(location):  # Verifica se chegou ao destino da localização.
                if (stop_mission): return

                if (lidar_can_move):
                    if(direction_of_correct == "direita"):
                        turn_right()
                    elif(direction_of_correct == "esquerda"):
                        turn_left()

                    go_forward()
                else:
                    runtime_log.info("Can't move, object in front of robot (lidar).")
                
                # Calculating robot position and correction to next itertion
                robot_location = Point(robot_latitude, robot_longitude)
                closest_point, correction_point = points_to_reach.get_closest_points(
                    robot_location,
                    correction_point_distance
                )

                # Path to the closest point
                path_closest = PathCorrection(robot_location, closest_point, 1)

                # Direction to go back to mission
                direction_of_correct: str = robot_location.get_correction_direction(
                    closest_point.latitude, 
                    closest_point.longitude, 
                    path_closest.angular_coff
                )
    print("Finalizou a missão")

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
    global stop_mission
    stop_mission = False
    print("Entrou no callback start mission")
    missions = Missions()
    missions.load_mission_file()
    run(missions)


def callback_stop_mission(dt: String):
    """
    Stop an mission that is being executed.
    """
    print("Entrou no callback stop mission")
    global stop_mission
    stop_mission = False


def callback_gps(dt: Coords) -> None:
    """
    Update current robot latitude and longitude
    """
    global robot_latitude, robot_longitude
    robot_latitude = round(float(dt.latitude), 5)
    robot_longitude = round(float(dt.longitude), 5)


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
