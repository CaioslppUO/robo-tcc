import rospy
from agrobot.msg import Control
import time

class ControlRobot:
    def __init__(self, pub: rospy.Publisher) -> None:
        self.__pub = pub
        self.__default_speed = 0.14
        self.__default_steer = 0.15
        self.__default_wait = 1.0

        self.__speed = 0.0
        self.__steer = 0.0
        self.__limit = 1.0
        self.__left_limit = -2
        self.__right_limit = 2
        self.__current_limit = 0
        self.begin = False

    def __send(self) -> None:
        """
        Send commands to the robot.
        """
        command = Control()
        command.speed = self.__speed
        command.steer = self.__steer
        command.limit = self.__limit
        self.__pub.publish(command)

    def forward(self) -> None:
        """
        Move the robot forward.
        """
        if(self.__current_limit > 0):
            self.left()
        elif(self.__current_limit < 0):
            self.right()
        self.__speed = self.__default_speed
        self.__steer = 0.0

    def left(self) -> None:
        """
        Turn the robot to the left.
        """
        if(self.__current_limit > self.__left_limit):
            self.__speed = self.__default_speed
            self.__steer = self.__default_steer
            time.sleep(self.__default_wait)
            self.__current_limit -= 1
        self.__speed = self.__default_speed
        self.__steer = 0.0

    def right(self) -> None:
        """
        Turn the robot to the right.
        """
        if(self.__current_limit < self.__right_limit):
            self.__speed = self.__default_speed
            self.__steer = -self.__default_steer
            time.sleep(self.__default_wait)
            self.__current_limit += 1
        self.__speed = self.__default_speed
        self.__steer = 0.0

    def stop(self) -> None:
        """
        Stop the robot.
        """
        self.__speed = 0.0
        self.__steer = 0.0

    def run(self) -> None:
        """
        Start the control module.
        """
        while True:
            if(self.begin):
                self.__send()
                time.sleep(0.1)
        