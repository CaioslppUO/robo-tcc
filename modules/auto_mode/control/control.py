import rospy
from agrobot.msg import Control
import time

class ControlRobot:
    def __init__(self, pub: rospy.Publisher) -> None:
        self.__pub = pub
        self.__default_speed = 0.16
        self.__default_steer = 0.3
        self.__default_wait = 1.0

        self.__speed = self.__default_speed
        self.__steer = 0.0
        self.__limit = 1.0

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
        self.__speed = self.__default_speed
        self.__steer = 0.0

    def left(self) -> None:
        """
        Turn the robot to the left.
        """
        self.__speed = self.__default_speed
        self.__steer = self.__default_steer
        time.sleep(self.__default_wait)
        self.__speed = self.__default_speed
        self.__steer = 0.0

    def right(self) -> None:
        """
        Turn the robot to the right.
        """
        self.__speed = self.__default_speed
        self.__steer = -self.__default_steer
        time.sleep(self.__default_wait)
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
        Start the control module by always going forward.
        """
        while True:
            self.__send()

    #def run_test(self) -> None:
    #    """
    #    Run a sequence of commands to test the robot.
    #    """
    #    print("Going forward...")
    #    self.forward()
    #    print("Turning left...")
    #    self.left()
    #    print("Turning right...")
    #    self.right()
    #    print("Going forward...")
    #    self.forward()
    #    print("Stopping")
    #    self.stop()
        