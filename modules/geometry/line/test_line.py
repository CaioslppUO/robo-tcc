from .line import Line
from .point import Point

class TestLine:
    
    def test_should_create_a_line(self):
        l = Line()
        assert len(l.points) == 0

    def test_should_add_points(self):
        l = Line()
        l.add(Point(-1, -1))
        l.add(Point(0, 0))
        l.add(Point(1, 1))
        assert l.points[0].get_point() == (-1, -1)
        assert l.points[1].get_point() == (0, 0)
        assert l.points[2].get_point() == (1, 1)
        assert l.points[0].id == 0
        assert l.points[1].id == 1
        assert l.points[2].id == 2

    def test_should_calculate_coefficients(self):
        l = Line()
        l.add(Point(-1, -1))
        l.add(Point(0, 0))
        assert l.angular_coefficient() == 1
        assert l.linear_coefficient() == 0

    def test_should_return_closest_point(self):
        l = Line()
        l.add(Point(0, 0))
        l.add(Point(20, 20))
        l.add(Point(-20, -20))
        assert l.closest_point(Point(1, 1)).get_point() == (0, 0)
        assert l.closest_point(Point(14, 14)).get_point() == (20, 20)
        assert l.closest_point(Point(-14, -8)).get_point() == (-20, -20)

    def test_should_detect_point_above_line(self):
        l = Line()
        l.add(Point(0, 0))
        l.add(Point(1, 1))
        assert l.is_above_the_line(Point(-1, -1)) == False
        assert l.is_above_the_line(Point(0, 0)) == False
        assert l.is_above_the_line(Point(1, 1)) == False
        assert l.is_above_the_line(Point(2, 2)) == False
        assert l.is_above_the_line(Point(3, 2)) == True
        assert l.is_above_the_line(Point(2, 3)) == False
        assert l.is_above_the_line(Point(0, 1)) == False
        assert l.is_above_the_line(Point(1, 0)) == True
        assert l.is_above_the_line(Point(0, -1)) == True
        assert l.is_above_the_line(Point(-1, 0)) == False