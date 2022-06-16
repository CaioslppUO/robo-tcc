from process_lidar import ProcessLidar

class TestProcessLidar():

    def test_should_select_9_points_array(self):
        points: list = [  0.10,  0.20,  0.30,  0.40,  0.50,
                          0.60,  0.70,  0.80,  0.90, 0.100, 
                         0.110, 0.120, 0.130, 0.140, 0.150,
                         0.160
                        ]
        range: int = 5
        center_point: int = 0
        processes_lidar = ProcessLidar()
        res = processes_lidar.select_points(points, range, center_point)
        assert len(res) == 9
        assert res == [0.10, 0.20, 0.160, 0.30, 0.150, 0.40, 0.140, 0.50, 0.130]

    def test_should_return_minimum_value_from_array(self):
        points: list = [  0.10,  0.20,  0.30,  0.40,  0.50,
                          0.60,  0.70,  0.80,  0.90, 0.100, 
                         0.110, 0.120, 0.130, 0.140, 0.150,
                         0.160
                        ]
        processes_lidar = ProcessLidar()
        assert processes_lidar.get_closest_distance(points) == 0.10