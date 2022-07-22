from path_calculator import PathCalculator, Point


class TestPathCalculator:
    def test_should_create_5_points_up_right(self):
        start = Point(-25.435348, -54.596970)
        end = Point(-25.435324, -54.596960)

        path = PathCalculator(start, end, 5)
        points = path.get_points_between()

        longitudes = points.get_longitudes()
        latitudes = points.get_latitudes()

        ## Interval = 0,0000025
        assert longitudes[0] == -54.596970
        assert longitudes[1] == -54.5969675
        assert longitudes[2] == -54.596965
        assert longitudes[3] == -54.5969625
        assert longitudes[4] == -54.59696

        assert latitudes[0] == -25.435348
        assert latitudes[1] == -25.435342
        assert latitudes[2] == -25.435336
        assert latitudes[3] == -25.43533
        assert latitudes[4] == -25.435324