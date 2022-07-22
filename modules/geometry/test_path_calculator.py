from .path_calculator import PathCalculator, Point, Line

class TestPathCalculator:

    def test_should_return_correct_direction_for_same_angular_coefficient_mission_quadrant_1(self):
        # Creating the mission
        mission = Line()
        mission.add(Point(0, 0))
        mission.add(Point(10, 10))

        # Creating the robot above the line quadrant 1
        robot_above_1 = Line()
        robot_above_1.add(Point(3, 2))
        robot_above_1.add(Point(4, 3))

        # Creating the robot under the line quadrant 1
        robot_under_1 = Line()
        robot_under_1.add(Point(2, 3))
        robot_under_1.add(Point(3, 4))

        # Creating the robot above the line quadrant 3
        robot_above_3 = Line()
        robot_above_3.add(Point(4, 3))
        robot_above_3.add(Point(3, 2))

        # Creating the robot under the line quadrant 3
        robot_under_3 = Line()
        robot_under_3.add(Point(3, 4))
        robot_under_3.add(Point(2, 3))

        # Creating the path calculator.
        path = PathCalculator(mission)

        # Correction Directions
        direction_above_1 = path.get_correction_direction(robot_above_1, robot_above_1.points[-1])
        direction_under_1 = path.get_correction_direction(robot_under_1, robot_under_1.points[-1])
        direction_above_3 = path.get_correction_direction(robot_above_3, robot_above_3.points[-1])
        direction_under_3 = path.get_correction_direction(robot_under_3, robot_under_3.points[-1])

        assert direction_above_1 == "right"
        assert direction_above_3 == "left"
        assert direction_under_1 == "left"
        assert direction_under_3 == "right"