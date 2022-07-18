from point import Point
from path_calculator import PathCalculator
from points import Points

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

    def test_should_correct_direction_up_right(self):
        start = Point(-25.435348, -54.596970)
        end = Point(-25.435324, -54.596960)
        path = PathCalculator(start, end, 15)
        points = path.get_points_between()

        robot_points = Points()
        robot_points.add_point(-25.435348, -54.596970)
        robot_points.add_point(-25.435345, -54.596970)
        robot_points.add_point(-25.435348, -54.596969)

        correction_0 = robot_points.get_point(0).get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        correction_1 = robot_points.get_point(1).get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        correction_2 = robot_points.get_point(2).get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        
        assert correction_0 == "forward"
        assert correction_1 == "right"
        assert correction_2 == "left"