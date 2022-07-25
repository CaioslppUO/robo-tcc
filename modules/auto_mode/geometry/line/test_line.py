from .line import Line
from .point import Point

class TestLine:
    
    def test_should_create_a_line(self):
        l = Line(Point(0, 0), Point(1, 1))
        assert l.p1.get_point() == (0, 0)
        assert l.p2.get_point() == (1, 1)

    def test_should_calculate_coefficients(self):
        l = Line(Point(-1, -1), Point(0, 0))
        assert l.angular_coefficient == 1
        assert l.linear_coefficient == 0

    def test_should_detect_point_above_line(self):
        l = Line(Point(0, 0), Point(1, 1))
        assert l.is_above(Point(-1, -1)) == False
        assert l.is_above(Point(0, 0)) == False
        assert l.is_above(Point(1, 1)) == False
        assert l.is_above(Point(2, 2)) == False
        assert l.is_above(Point(3, 2)) == True
        assert l.is_above(Point(2, 3)) == False
        assert l.is_above(Point(0, 1)) == False
        assert l.is_above(Point(1, 0)) == True
        assert l.is_above(Point(0, -1)) == True
        assert l.is_above(Point(-1, 0)) == False

    def test_should_detect_point_in_line(self):
        l = Line(Point(0, 0), Point(1, 1))
        assert l.is_in(Point(2, 2)) == True
        assert l.is_in(Point(-1, -1)) == True
        assert l.is_in(Point(1, 2)) == False
        assert l.is_in(Point(2, 1)) == False

    def test_should_detect_point_under_line(self):
        l = Line(Point(0, 0), Point(1, 1))
        assert l.is_under(Point(0, 1)) == True
        assert l.is_under(Point(1, 0)) == False
        assert l.is_under(Point(1, 2)) == True
        assert l.is_under(Point(-1, 1)) == True
        assert l.is_under(Point(-1, -2)) == False

    def test_should_return_the_quadrants(self):
        l = Line(Point(0, 0), Point(1, 1))
        assert l.quadrant == 1
        l = Line(Point(1, 1), Point(0, 0))
        assert l.quadrant == 3
        l = Line(Point(0, 0), Point(1, -1))
        assert l.quadrant == 4
        l = Line(Point(0, 0), Point(-1, 1))
        assert l.quadrant == 2

    def test_should_calculate_slopes_as_degrees_and_degrees_as_slope(self):
        # Quadrant 1
        l1 = Line(Point(0, 0), Point(1, 1))
        l2 = Line(Point(0, 0), Point(90000, 1))
        l3 = Line(Point(0, 0), Point(1, 90000))

        assert l1.slope_to_degrees() == 45.00
        assert l1.degrees_to_slope(l1.slope_to_degrees()) == l1.angular_coefficient
        assert l2.slope_to_degrees() == 89.9993634
        assert l2.degrees_to_slope(l2.slope_to_degrees()) == 90002.7953341
        assert l3.slope_to_degrees() == 0.000636
        assert l3.degrees_to_slope(l3.slope_to_degrees()) == l3.angular_coefficient

        # Quadrant 2
        l4 = Line(Point(0, 0), Point(-1, 1))
        l5 = Line(Point(0, 0), Point(-1, 90000))
        l6 = Line(Point(0, 0), Point(-90000, 1))

        assert l4.slope_to_degrees() == -45.00
        assert l4.degrees_to_slope(l4.slope_to_degrees()) == l4.angular_coefficient
        assert l5.slope_to_degrees() == -0.000636
        assert l5.degrees_to_slope(l5.slope_to_degrees()) == l5.angular_coefficient
        assert l6.slope_to_degrees() == -89.9993634
        assert l6.degrees_to_slope(l6.slope_to_degrees()) == -90002.7953341

        # Quadrant 3
        l7 = Line(Point(0, 0), Point(-1, -1))
        l8 = Line(Point(0, 0), Point(-90000, -1))
        l9 = Line(Point(0, 0), Point(-1, -90000))

        assert l7.slope_to_degrees() == 45.00
        assert l7.degrees_to_slope(l7.slope_to_degrees()) == l7.angular_coefficient
        assert l8.slope_to_degrees() == 89.9993634
        assert l8.degrees_to_slope(l8.slope_to_degrees()) == 90002.7953341
        assert l9.slope_to_degrees() == 0.000636
        assert l9.degrees_to_slope(l9.slope_to_degrees()) == l9.angular_coefficient

        # Quadrant 4
        l10 = Line(Point(0, 0), Point(1, -1))
        l11 = Line(Point(0, 0), Point(90000, -1))
        l12 = Line(Point(0, 0), Point(1, -90000))

        assert l10.slope_to_degrees() == -45.00
        assert l10.degrees_to_slope(l10.slope_to_degrees()) == l10.angular_coefficient
        assert l11.slope_to_degrees() == -89.9993634
        assert l11.degrees_to_slope(l11.slope_to_degrees()) == -90002.7953341
        assert l12.slope_to_degrees() == -0.000636
        assert l12.degrees_to_slope(l12.slope_to_degrees()) == l12.angular_coefficient

    def test_should_increase_slope_until_complete_lap(self):
        increment = 15
        l = Line(Point(0, 0), Point(1, 0))
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        assert l.quadrant == 2
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        assert l.quadrant == 3
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        assert l.quadrant == 4
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        l.clockwise_slope(increment)
        assert l.quadrant == 1

    def test_should_increase_slope_until_complete_lap(self):
        decrement = 15
        l = Line(Point(0, 0), Point(1, 0))
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        assert l.quadrant == 4
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        assert l.quadrant == 3
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        assert l.quadrant == 2
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        l.counter_clockwise_slope(decrement)
        assert l.quadrant == 1

    def test_should_detect_correct_rotation_direction_mission_quadrant_1(self):
        mission = Line(Point(0, 0), Point(5, 5))

        assert mission.quadrant == 1
        assert mission.angular_coefficient > 0

        # Robot Under The Mission Line
        robot_under_line = Point(0, 1)
        correction_line = Line(robot_under_line, Point(2, 2))

        robot_under_direction_under_q2 = Line(robot_under_line, Point(-2, 3))
        robot_under_direction_under_q4 = Line(robot_under_line, Point(2, -3))
        
        robot_under_direction_above_q1 = Line(robot_under_line, Point(7, 4))
        robot_under_direction_above_q3 = Line(robot_under_line, Point(-12, -4))

        robot_under_direction_in_q1 = Line(robot_under_line, Point(2, 2))
        robot_under_direction_in_q3 = Line(robot_under_line, Point(-4, -1))

        # Lines Verifications
        assert mission.is_under(robot_under_line) == True
        assert correction_line.quadrant == 1
        assert correction_line.angular_coefficient >= 0
        assert robot_under_direction_under_q2.quadrant == 2
        assert robot_under_direction_under_q2.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_under_q4.quadrant == 4
        assert robot_under_direction_under_q4.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_above_q1.quadrant == 1
        assert robot_under_direction_above_q1.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_above_q3.quadrant == 3
        assert robot_under_direction_above_q3.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_in_q1.quadrant == 1
        assert robot_under_direction_in_q1.angular_coefficient == correction_line.angular_coefficient
        assert robot_under_direction_in_q3.quadrant == 3
        assert robot_under_direction_in_q1.angular_coefficient == correction_line.angular_coefficient
        
        # Correct Directions
        assert robot_under_direction_under_q2.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_under_q4.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_above_q1.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_above_q3.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_in_q1.get_smaller_rotation_direction(correction_line) == "none"
        assert robot_under_direction_in_q3.get_smaller_rotation_direction(correction_line) == "clockwise"

        # Robot Above the Mission Line
        robot_above_line = Point(2, 1)
        correction_line_2 = Line(robot_above_line, Point(3, 3))

        robot_above_direction_under_q2 = Line(robot_above_line, Point(0, 3))
        robot_above_direction_under_q4 = Line(robot_above_line, Point(4, -1))

        robot_above_direction_above_q1 = Line(robot_above_line, Point(4, 2))
        robot_above_direction_above_q3 = Line(robot_above_line, Point(-2, -1))

        robot_above_direction_in_q1 = Line(robot_above_line, Point(3, 3))
        robot_above_direction_in_q3 = Line(robot_above_line, Point(-1, -5))

        # Lines Verifications
        assert mission.is_above(robot_above_line) == True
        assert correction_line_2.quadrant == 1
        assert correction_line_2.angular_coefficient >= 0
        assert robot_above_direction_under_q2.quadrant == 2
        assert robot_above_direction_under_q2.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_under_q4.quadrant == 4
        assert robot_above_direction_under_q4.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_above_q1.quadrant == 1
        assert robot_above_direction_above_q1.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_above_q3.quadrant == 3
        assert robot_above_direction_above_q3.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_in_q1.quadrant == 1
        assert robot_above_direction_in_q1.angular_coefficient == correction_line_2.angular_coefficient
        assert robot_above_direction_in_q3.quadrant == 3
        assert robot_above_direction_in_q3.angular_coefficient == correction_line_2.angular_coefficient

        # Correct Directions
        assert robot_above_direction_under_q2.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_under_q4.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_above_q1.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_above_q3.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_in_q1.get_smaller_rotation_direction(correction_line_2) == "none"
        assert robot_above_direction_in_q3.get_smaller_rotation_direction(correction_line_2) == "clockwise"

    def test_should_detect_correct_rotation_direction_in_mission_quadrant_2(self):
        mission = Line(Point(0, 0), Point(-5, 5))

        assert mission.quadrant == 2
        assert mission.angular_coefficient < 0

        # Robot Under the Mission Line
        robot_under_line = Point(-2, 1)
        correction_line = Line(robot_under_line, Point(-3, 3))

        robot_under_direction_above_q1 = Line(robot_under_line, Point(0, 3))
        robot_under_direction_above_q3 = Line(robot_under_line, Point(-3, -1))

        robot_under_direction_under_q2 = Line(robot_under_line, Point(-4, 2))
        robot_under_direction_under_q4 = Line(robot_under_line, Point(2, -2))

        robot_under_direction_in_q2 = Line(robot_under_line, Point(-3, 3))
        robot_under_direction_in_q4 = Line(robot_under_line, Point(1, -5))

        # Lines Verifications
        assert mission.is_under(robot_under_line) == True
        assert correction_line.quadrant == 2
        assert correction_line.angular_coefficient < 0
        assert robot_under_direction_above_q1.quadrant == 1
        assert robot_under_direction_above_q1.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_above_q3.quadrant == 3
        assert robot_under_direction_above_q3.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_under_q2.quadrant == 2
        assert robot_under_direction_under_q2.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_under_q4.quadrant == 4
        assert robot_under_direction_under_q4.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_in_q2.quadrant == 2
        assert robot_under_direction_in_q2.angular_coefficient == correction_line.angular_coefficient
        assert robot_under_direction_in_q4.quadrant == 4
        assert robot_under_direction_in_q4.angular_coefficient == correction_line.angular_coefficient

        # Correct Directions
        assert robot_under_direction_above_q1.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_above_q3.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_under_q2.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_under_q4.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_in_q2.get_smaller_rotation_direction(correction_line) == "none"
        assert robot_under_direction_in_q4.get_smaller_rotation_direction(correction_line) == "clockwise"

        # Robot Above the Mission Line
        robot_above_line = Point(-1, 2)
        correction_line_2 = Line(robot_above_line, Point(-3, 4))

        robot_above_direction_above_q1 = Line(robot_above_line, Point(0, 3))
        robot_above_direction_above_q3 = Line(robot_above_line, Point(-2, 1))

        robot_above_direction_under_q2 = Line(robot_above_line, Point(-3, 3))
        robot_above_direction_under_q4 = Line(robot_above_line, Point(4, -2))

        robot_above_direction_in_q2 = Line(robot_above_line, Point(-3, 4))
        robot_above_direction_in_q4 = Line(robot_above_line, Point(3, -2))

        assert mission.is_above(robot_above_line) == True
        assert correction_line_2.quadrant == 2
        assert correction_line_2.angular_coefficient < 0

        # Line Verifications
        assert robot_above_direction_above_q1.quadrant == 1
        assert robot_above_direction_above_q1.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_above_q3.quadrant == 3
        assert robot_above_direction_above_q3.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_under_q2.quadrant == 2
        assert robot_above_direction_under_q2.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_under_q4.quadrant == 4
        assert robot_above_direction_under_q4.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_in_q2.quadrant == 2
        assert robot_above_direction_in_q2.angular_coefficient == correction_line_2.angular_coefficient
        assert robot_above_direction_in_q4.quadrant == 4
        assert robot_above_direction_in_q4.angular_coefficient == correction_line_2.angular_coefficient

        # Correct Directions
        assert robot_above_direction_above_q1.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_above_q3.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_under_q2.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_under_q4.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_in_q2.get_smaller_rotation_direction(correction_line_2) == "none"
        assert robot_above_direction_in_q4.get_smaller_rotation_direction(correction_line_2) == "clockwise"

    def test_should_detect_correct_rotation_direction_in_mission_quadrant_3(self):
        mission = Line(Point(0, 0), Point(-5, -5))

        assert mission.quadrant == 3
        assert mission.angular_coefficient > 0

        # Robot Under Line
        robot_under_line = Point(-2, -1)
        correction_line = Line(robot_under_line, Point(-3, -3))

        robot_under_direction_above_q1 = Line(robot_under_line, Point(1, 1))
        robot_under_direction_above_q3 = Line(robot_under_line, Point(-3, -2))

        robot_under_direction_under_q2 = Line(robot_under_line, Point(-3, 1))
        robot_under_direction_under_q4 = Line(robot_under_line, Point(2, -3))

        robot_under_direction_in_q1 = Line(robot_under_line, Point(0.5, 4))
        robot_under_direction_in_q3 = Line(robot_under_line, Point(-3, -3))

        # Line Verifications
        assert mission.is_under(robot_under_line)
        assert correction_line.quadrant == 3
        assert correction_line.angular_coefficient > 0
        assert robot_under_direction_above_q1.quadrant == 1
        assert robot_under_direction_above_q1.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_above_q3.quadrant == 3
        assert robot_under_direction_above_q3.angular_coefficient > correction_line.angular_coefficient
        assert robot_under_direction_under_q2.quadrant == 2
        assert robot_under_direction_under_q2.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_under_q4.quadrant == 4
        assert robot_under_direction_under_q4.angular_coefficient < correction_line.angular_coefficient
        assert robot_under_direction_in_q1.quadrant == 1
        assert robot_under_direction_in_q1.angular_coefficient == correction_line.angular_coefficient
        assert robot_under_direction_in_q3.quadrant == 3
        assert robot_under_direction_in_q3.angular_coefficient == correction_line.angular_coefficient

        # Correct Direction
        assert robot_under_direction_above_q1.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_above_q3.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_under_q2.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_under_q4.get_smaller_rotation_direction(correction_line) == "counter_clockwise"
        assert robot_under_direction_in_q1.get_smaller_rotation_direction(correction_line) == "clockwise"
        assert robot_under_direction_in_q3.get_smaller_rotation_direction(correction_line) == "none"

        # Robot Above The Mission Line
        robot_above_line = Point(-1, -2)
        correction_line_2 = Line(robot_above_line, Point(-3, -3))

        robot_above_direction_above_q1 = Line(robot_above_line, Point(10, 2))
        robot_above_direction_above_q3 = Line(robot_above_line, Point(-10, -4))

        robot_above_direction_under_q2 = Line(robot_above_line, Point(-4, 1))
        robot_above_direction_under_q4 = Line(robot_above_line, Point(2, -3))

        robot_above_direction_in_q1 = Line(robot_above_line, Point(11, 4))
        robot_above_direction_in_q3 = Line(robot_above_line, Point(-3, -3))

        # Line Verifications
        assert mission.is_above(robot_above_line) == True
        assert correction_line_2.quadrant == 3
        assert correction_line_2.angular_coefficient > 0
        assert robot_above_direction_above_q1.quadrant == 1
        assert robot_above_direction_above_q1.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_above_q3.quadrant == 3
        assert robot_above_direction_above_q3.angular_coefficient > correction_line_2.angular_coefficient
        assert robot_above_direction_under_q2.quadrant == 2
        assert robot_above_direction_under_q2.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_under_q4.quadrant == 4
        assert robot_above_direction_under_q4.angular_coefficient < correction_line_2.angular_coefficient
        assert robot_above_direction_in_q1.quadrant == 1
        assert robot_above_direction_in_q1.angular_coefficient == correction_line_2.angular_coefficient
        assert robot_above_direction_in_q3.quadrant == 3
        assert robot_above_direction_in_q3.angular_coefficient == correction_line_2.angular_coefficient

        # Correct Direction
        assert robot_above_direction_above_q1.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_above_q3.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_under_q2.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_under_q4.get_smaller_rotation_direction(correction_line_2) == "counter_clockwise"
        assert robot_above_direction_in_q1.get_smaller_rotation_direction(correction_line_2) == "clockwise"
        assert robot_above_direction_in_q3.get_smaller_rotation_direction(correction_line_2) == "none"