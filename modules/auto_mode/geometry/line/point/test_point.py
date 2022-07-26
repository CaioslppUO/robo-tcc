from .point import Point

class TestPoint:

    def test_should_create_a_point(self):
        p = Point(-1, 15)
        assert p.latitude == -1
        assert p.longitude == 15

    def test_should_return_a_point(self):
        p1 = Point(-2, -15)
        assert p1.get_point() == (-2, -15)

    def test_should_compare_two_points(self):
        p1 = Point(-1, 15)
        p2 = Point(15, -1)
        assert p1.equal(p1.latitude, p1.longitude) == True
        assert p2.equal(p2.latitude, p2.longitude) == True
        assert p1.equal(p2.latitude, p2.longitude) == False
        assert p2.equal(p1.latitude, p1.longitude) == False

    def test_should_calculate_difference_between_points(self):
        p1 = Point(10, 15)
        p2 = Point(5, 20)
        diff = p1.difference(p2.latitude, p2.longitude)
        assert diff.latitude == -5
        assert diff.longitude == 5

    def test_should_calculate_distance_between_points(self):
        p1 = Point(0, 0)
        p2 = Point(0, 0)
        p3 = Point(1, 1)
        p4 = Point(2, 2)
        p5 = Point(-1, -1)
        assert p1.distance(p1.latitude, p1.longitude) == 0
        assert p1.distance(p2.latitude, p2.longitude) == 0
        assert p2.distance(p2.latitude, p2.longitude) == 0
        assert p2.distance(p1.latitude, p1.longitude) == 0
        assert p1.distance(p3.latitude, p3.longitude) == 1.4142136
        assert p1.distance(p4.latitude, p4.longitude) == 2.8284271
        assert p1.distance(p5.latitude, p5.longitude) == 1.4142136
        assert p3.distance(p4.latitude, p4.longitude) == 1.4142136