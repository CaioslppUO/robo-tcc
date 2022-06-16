"""
@package control_movement.py
Manage the robot movement permission based on the lidar object detection.
"""

from object_detector import ObjectDetector

class ControlMovement:
    def __init__(self, secure_distance: int) -> None:
        self.object_detector = ObjectDetector(secure_distance)

    def can_move(self) -> bool:
        """
        Return if the robot can or cannot move.
        """
        if (
            self.object_detector.has_object_in_front()
            or
            self.object_detector.has_object_in_left()
            or
            self.object_detector.has_object_in_right()
        ):
            return False
        return True