from .line import Line, Point

class PathCalculator:
    def __init__(self, mission: Line) -> None:
        self.mission: Line = mission

    def get_correction_direction(self, robot_direction: Line, robot_point: Point) -> str:
        """
        Calculate the correction direction based on robot direction and mission direction.
        """
        # Angular Coefficients
        mission_angular_coefficient = self.mission.angular_coefficient()
        robot_angular_coefficient = robot_direction.angular_coefficient()
        
        # Robot Quadrant
        robot_quadrant = robot_direction.quadrant()
        mission_quadrant = self.mission.quadrant()

        # Robot Position Relative to the Mission Line
        above_the_line = self.mission.is_above_the_line(robot_point)
        in_the_line = self.mission.is_in_the_line(robot_point)
        under_the_line = self.mission.is_under_the_line(robot_point)

        # Slope Relations
        increasing_slope = robot_angular_coefficient - mission_angular_coefficient > 0
        decreasing_slope = robot_angular_coefficient - mission_angular_coefficient < 0
        equal_slope = robot_angular_coefficient - mission_angular_coefficient == 0

        if(in_the_line):
            return "forward"

        if(mission_quadrant == 1):
            if(above_the_line):
                if(equal_slope):
                    return "right"
                if(increasing_slope):
                    if(robot_quadrant == 1):
                        return "right"
                    elif(robot_quadrant == 3):
                        return "left"
                    else:
                        raise Exception("Robot above the line, slope positive but quadrant is not 1 or 3")
                if(decreasing_slope):
                    if(robot_quadrant == 2):
                        return "forward"
                    elif(robot_quadrant == 4):
                        return "left"
                    else:
                        raise Exception("Robot above the line, slope negative but quadrant is not 2 or 4")
            if(under_the_line):
                if(equal_slope):
                    return "left"
                if(increasing_slope):
                    if(robot_quadrant == 1):
                        return "forward"
                    elif(robot_quadrant == 3):
                        return "right"
                    else:
                        raise Exception("Robot under the line, slope positive but quadrant is not 1 or 3")
                if(decreasing_slope):
                    if(robot_quadrant == 1 or robot_quadrant == 2):
                        return "left"
                    elif(robot_quadrant == 4):
                        return "right"
                    else:
                        raise Exception("Robot under the line, slope negative but quadrant is not 1, 2 or 4")

