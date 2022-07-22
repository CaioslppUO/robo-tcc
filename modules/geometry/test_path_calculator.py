from .path_calculator import PathCalculator, Point, Line

class TestPathCalculator:

    def test_should_return_correct_direction_for_same_angular_coefficient_mission_quadrant_1(self):
        # Creating the mission
        mission = Line()
        mission.add(Point(0, 0))
        mission.add(Point(10, 10))

        # Creating the robot above the line
        robot_above = Line()
        robot_above.add(Point(3, 2))
        robot_above.add(Point(4, 3))

        # Creating the robot under the line
        robot_under = Line()
        robot_under.add(Point(2, 3))
        robot_under.add(Point(3, 4))

        # Creating the path calculator.
        path = PathCalculator(mission)

        # Correction Directions
        direction_above = path.get_correction_direction(robot_above, robot_above.points[-1])
        direction_under = path.get_correction_direction(robot_under, robot_under.points[-1])

        assert direction_above == "right"
        assert direction_under == "left"