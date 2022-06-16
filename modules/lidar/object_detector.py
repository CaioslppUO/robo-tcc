"""
@package object_detector.py
Manage the object detection.
"""

class ObjectDetector:
    INFINITE = 99999999999999999999999999999

    def __init__(self, secure_distance: int) -> None:
        self.secure_distance = secure_distance
        self.front = self.INFINITE
        self.left = self.INFINITE
        self.right = self.INFINITE

    def set_front(self, value: int) -> None:
        """
        Update the value of the front attribute.
        """
        self.front = value

    def set_left(self, value: int) -> None:
        """
        Update the value of the left attribute.
        """
        self.left = value

    def set_right(self, value: int) -> None:
        """
        Update the value of the right attribute.
        """
        self.right = value

    def has_object_in_front(self) -> bool:
        """
        Verify if there is an object in front of the robot.
        """
        if(self.front <= self.secure_distance):
            return True
        return False

    def has_object_in_left(self) -> bool:
        """
        Verify if there is an object in left of the robot.
        """
        if(self.left <= self.secure_distance):
            return True
        return False

    def has_object_in_right(self) -> bool:
       """
       Verify if there is an object in right of the robot.
       """
       if(self.right <= self.secure_distance):
           return True
       return False