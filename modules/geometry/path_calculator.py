from .line import Line, Point

class PathCalculator:
    def __init__(self, mission: Line) -> None:
        self.mission: Line = mission

   