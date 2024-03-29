#!/usr/bin/env python3

"""
@package get_robot_commands
Get control data from app server and send it to ROS server.
"""

import rospy
import pathlib
import traceback
from agrobot_services.log import Log
from agrobot.msg import Control
from shutil import which
from agrobot_services.runtime_log import RuntimeLog
import socketio
from std_msgs.msg import Bool, String
from pathlib import Path
import sys
import json

# Directory variables
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

# get_robot_commands node
rospy.init_node('get_robot_commands', anonymous=True)

# Log class
log: Log = Log("get_robot_commands.py")
runtime_log: RuntimeLog = RuntimeLog("get_robot_commands.py")

# Command sending control
last_command = None

# Socketio connection handler
sio = socketio.Client()


def publish_command(command: Control) -> None:
    """
    Publish the command to get_robot_commands topic.
    """
    global last_command
    try:
        if(last_command != command):
            pub = rospy.Publisher("/get_robot_commands",
                                  Control, queue_size=10)
            pub.publish(command)
            last_command = command
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error(
            "Could not publish new command to /get_robot_commands")


def publish_module_activated(command: Bool) -> None:
    """
    Publish the command to module_activated topic.
    """
    try:
        pub = rospy.Publisher("/module_activated", Bool, queue_size=10)
        pub.publish(command)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not publish new command to /module_activated")


def publish_mode_changed(mode: str) -> None:
    """
    Publish the command to change_mode topic.
    """
    try:
        pub = rospy.Publisher("/change_mode", String, queue_size=10)
        pub.publish(mode)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not publish new command to /change_mode")


@sio.on("control_update_changed")
def setup_command(command) -> None:
    """
    Separate and assemble the command to control the robot.

    Parameters:
    command -> Json content with speed and steer values.
    """
    try:
        cm: Control = Control()
        cm.speed = float(command["speed"]) * float(command["limit"])
        cm.steer = float(command["steer"])
        cm.limit = float(command["limit"])
        publish_command(cm)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not setup Control command")


@sio.on("module_activated_changed")
def setup_module_activated(command) -> None:
    """
    Separate and assemble the command to motor the robot.

    Parameters:
    command -> Bool content with state motor..
    """
    try:
        publish_module_activated(Bool(command))
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not setup power motor command")


@sio.on("control_mode_changed")
def setup_module_activated(mode: str) -> None:
    """
    Update in which mode the robot should turn (A, B, C).

    Parameters:
    mode -> The mode in which the robot should turn (A, B, C)
    """
    try:
        publish_mode_changed(mode)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not setup power motor command")


@sio.on("mission_changed")
def setup_mission_changed(mission) -> None:
    aux = sys.argv[0].split("/")[0:-1]
    pth = ""
    for i in aux:
        pth += i + "/"
    mission_file = Path(pth).absolute().joinpath(
        "modules/mission_control/mission.json")
    with open(mission_file, "w") as file:
        json.dump(mission, file)


def connect():
    sio.connect('http://localhost:3000')


if __name__ == '__main__':
    try:
        connected = False
        while(not connected):
            try:
                connect()
                connected = True
            except Exception as e:
                runtime_log.info("Tentando reconexão - get_robot_commands")
    except rospy.ROSInterruptException:
        log.warning("Roscore was interrupted.")
