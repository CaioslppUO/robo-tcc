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
