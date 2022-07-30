"""
@package mission_data.py
Represent the structure used to store mission log files during the execution of a mission.
"""

class Mission:
    """
    Represent the Mission data.
    """
    def __init__(self, mission_name: str, mission_line: "tuple[tuple[float, float], tuple[float, float]]", mission_points: "list[tuple[float, float]]") -> None:
        self.name = mission_name
        self.line = mission_line
        self.points = mission_points
        
class Correction:
    """
    Represent the Correction data.
    """
    def __init__(self, correction_line: "tuple[tuple[float, float], tuple[float, float]]", correction_direction: str, error: float, chooser: int) -> None:
        self.line = correction_line
        self.direction = correction_direction
        self.error = error
        self.chooser = chooser

class Robot:
    """
    Represent the Robot data.
    """
    def __init__(self, robot_pos: "tuple[float, float]", robot_direction_line: "tuple[tuple[float, float], tuple[float, float]]") -> None:
        self.robot_pos = robot_pos
        self.direction_line = robot_direction_line

class MissionRecord:
    """
    Represent a Mission Record (Loop Iteration).
    """
    def __init__(self, iteration: int, mission: Mission, correction: Correction, robot: Robot) -> None:
        self.iteration = iteration
        self.mission = mission
        self.correction = correction
        self.robot = robot