"""
@package control.py
Control robot direction and movement.
"""

import rospy
from agrobot.msg import Control

class ControlRobot:
    def __init__(self, pub: rospy.Publisher) -> None:
        self.__pub = pub
        self.__timeout = 50 # Loops running each command
        self.__default_speed = 0.15
        self.__default_steer = 0.3

    def __send(self, speed: float, steer: float, limit: float = 1.0) -> None:
        """
        Send commands to the robot.
        """
        command = Control()
        command.speed = speed
        command.steer = steer
        command.limit = limit
        for _ in range(0, self.__timeout):
            self.__pub.publish(command)

    def go_forward(self) -> None:
        """
        Move the robot forward.
        """
        self.__send(self.__default_speed, 0.0)

    def turn_left(self) -> None:
        """
        Turn the robot to the left.
        """
        self.__send(0.0, -self.__default_steer)

    def turn_right(self) -> None:
        """
        Turn the robot to the right.
        """
        self.__send(0.0, self.__default_steer)

    def stop(self) -> None:
        """
        Stop the robot.
        """
        self.__send(0.0, 0.0, 0.0)