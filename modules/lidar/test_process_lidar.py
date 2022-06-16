from process_lidar import ProcessLidar

class TestProcessLidar():

    def test_should_select_9_points_array(self):
        points: list = [  10,  20,  30,  40,  50,
                          60,  70,  80,  90, 100, 
                         110, 120, 130, 140, 150,
                         160
                        ]
        range: int = 5
        center_point: int = 0
        processes_lidar = ProcessLidar()
        res = processes_lidar.select_points(points, range, center_point)
        assert len(res) == 9
        assert res == [10, 20, 160, 30, 150, 40, 140, 50, 130]

    def test_should_return_minimum_value_from_array(self):
        points: list = [  10,  20,  30,  40,  50,
                          60,  70,  80,  90, 100, 
                         110, 120, 130, 140, 150,
                         160
                        ]
        processes_lidar = ProcessLidar()
        assert processes_lidar.get_closest_distance(points) == 10