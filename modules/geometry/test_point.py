from point import Point

class TestPoint():

    def test_should_create_a_point(self):
        point = Point(-1, 1, 6)
        assert point.latitude == -1
        assert point.longitude == 1

    def test_should_calculate_angular_coefficient(self):
        point = Point(0, 0, 6)
        assert point.get_angular_coefficient(Point(1, 1, 6)) == 1.000000
        assert point.get_angular_coefficient(Point(-1, -1, 6)) == 1.000000
        assert point.get_angular_coefficient(Point(-1, 1, 6)) == -1.000000
        assert point.get_angular_coefficient(Point(1, -1, 6)) == -1.000000

    def test_should_calculate_differences(self):
        point = Point(0, 0, 6)
        assert point.get_difference(Point(1, 1, 6)).latitude ==  1
        assert point.get_difference(Point(1, 1, 6)).longitude ==  1

        point = Point(5, 6, 6)
        assert point.get_difference(Point(1, 1, 6)).latitude ==  -4
        assert point.get_difference(Point(1, 1, 6)).longitude ==  -5

    def test_should_calculate_distances(self):
        point = Point(0, 0, 6)
        assert point.get_distance(Point(1, 1, 6)) == 1.414214
        assert point.get_distance(Point(2, 2, 6)) == 2.828427
        assert point.get_distance(Point(5, 5, 6)) == 7.071068