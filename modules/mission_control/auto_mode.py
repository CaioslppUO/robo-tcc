#!/usr/bin/env python
"""
@package mission.py
Control the automatic mode.
"""

import rospy
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
from mission import Missions

# Auto Mode node
rospy.init_node('auto_mode', anonymous=True)

# Log class
log: Log = Log("encoder.py")
runtime_log: RuntimeLog = RuntimeLog("encoder.py")

# Global control variables
stop_mission = False
lidar_can_move = True
robot_latitude = None
robot_longitude = None
robot_compass = None

def run(missions: Missions) -> None:
    """
    Execute the mission.
    """
    while(not stop_mission):
        for mission in missions.get_missions(): # Execute every mission
            if(stop_mission):
                return

            for location in mission.get_locations(): # Execute every location
                if(stop_mission):
                    return 

                while(location.latitude != robot_latitude and location.longitude != robot_longitude): # Verifica se chegou ao destino da localização.
                    if(stop_mission):
                        return None

                    if(lidar_can_move):
                        need_to_correct_route = True # Verificar se precisa ou não corrigir a direção
                        while(need_to_correct_route): # Lógica para corrigir a direção do robô
                            if(stop_mission):
                                return None
                                 
                        # Vai reto
                    else:
                        runtime_log.info("Can't move, object in front of robot (lidar).")

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

def callback_gps(dt: String) -> None:
    """
    Update current robot latitude and longitude
    """
    global robot_latitude, robot_longitude
    robot_latitude = float(dt.data.split(";")[0])
    robot_longitude = float(dt.data.split(";")[1])

def callback_compass(dt: String) -> None:
    """
    Update current robot latitude and longitude
    """
    global robot_compass
    robot_compass = float(dt.data)

if __name__ == "__main__":
    try:
        rospy.Subscriber('/lidar', String, callback_lidar)
        rospy.Subscriber('/gps', String, callback_gps)
        rospy.Subscriber('/compass', String, callback_compass)
        rospy.Subscriber('/start_mission', String, callback_start_mission)
        rospy.Subscriber('/stop_mission', String, callback_stop_mission)
        rospy.spin()
    except:
        runtime_log.error("Auto Mode died. Check logs file.")