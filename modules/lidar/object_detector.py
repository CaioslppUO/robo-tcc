"""
@package object_detector.py
Manage the object detection.
"""

class ObjectDetector:
    INFINITE = 99999999999999999999999999999.0

    def __init__(self, secure_distance: float) -> None:
        self.secure_distance = secure_distance
        self.distance = self.INFINITE

    def set_distance(self, value: float) -> None:
        """
        Update the value of the front attribute.
        """
        self.distance = value

    def has_object_in_front(self) -> bool:
        """
        Verify if there is an object in front of the robot.
        """
        if(self.distance <= self.secure_distance):
            return True
        return False