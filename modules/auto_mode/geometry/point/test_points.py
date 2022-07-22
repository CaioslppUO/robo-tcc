from points import Points
from point import Point

class TestPoint():

    def test_should_create_points(self):
        points = Points(6)
        points.add_point(0, 1)
        points.add_point(2, 3)
        points.add_point(4, 5)

        lat = points.get_latitudes()
        lon = points.get_longitudes()

        assert lat[0] == 0
        assert lat[1] == 2
        assert lat[2] == 4

        assert lon[0] == 1
        assert lon[1] == 3
        assert lon[2] == 5

    def test_should_return_closes_point(self):
        points = Points(6)
        points.add_point(0, 0)
        points.add_point(1, 1)
        points.add_point(2, 2)
        points.add_point(3, 3)
        points.add_point(4, 4)
        points.add_point(5, 5)

        a1, a2 = points.get_closest_points(Point(0.4, 0.4, 6), 2)
        assert a1.id ==  0
        assert a2.id == 2

        a1, a2 = points.get_closest_points(Point(0.5, 0.6, 6), 2)
        assert a1.id ==  1
        assert a2.id == 3

        a1, a2 = points.get_closest_points(Point(1.5, 0.6, 6), 2)
        assert a1.id ==  1
        assert a2.id == 3

        a1, a2 = points.get_closest_points(Point(2.5, 0.6, 6), 2)
        assert a1.id ==  2
        assert a2.id == 4

        a1, a2 = points.get_closest_points(Point(2.5, 2.6, 6), 2)
        assert a1.id ==  3
        assert a2.id == 5

        a1, a2 = points.get_closest_points(Point(4.5, 5, 6), 2)
        assert a1.id ==  5
        assert a2.id == 5