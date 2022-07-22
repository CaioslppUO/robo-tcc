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

    def test_should_calculate_slopes_as_degrees(self):
        # Quadrant 1
        l1 = Line(Point(0, 0), Point(1, 1))
        l2 = Line(Point(0, 0), Point(90000, 1))
        l3 = Line(Point(0, 0), Point(1, 90000))

        assert l1.slope_as_degrees() == 45.00
        assert l2.slope_as_degrees() == 90.00
        assert l3.slope_as_degrees() == 0.00

        # Quadrant 2
        l4 = Line(Point(0, 0), Point(-1, 1))
        l5 = Line(Point(0, 0), Point(-1, 90000))
        l6 = Line(Point(0, 0), Point(-90000, 1))

        assert l4.slope_as_degrees() == -45.00
        assert l5.slope_as_degrees() == -0.00
        assert l6.slope_as_degrees() == -90.00

        # Quadrant 3
        l7 = Line(Point(0, 0), Point(-1, -1))
        l8 = Line(Point(0, 0), Point(-90000, -1))
        l9 = Line(Point(0, 0), Point(-1, -90000))

        assert l7.slope_as_degrees() == 45.00
        assert l8.slope_as_degrees() == 90.00
        assert l9.slope_as_degrees() == 0.00

        # Quadrant 4
        l10 = Line(Point(0, 0), Point(1, -1))
        l11 = Line(Point(0, 0), Point(90000, -1))
        l12 = Line(Point(0, 0), Point(1, -90000))

        assert l10.slope_as_degrees() == -45.00
        assert l11.slope_as_degrees() == -90.00
        assert l12.slope_as_degrees() == -0.00