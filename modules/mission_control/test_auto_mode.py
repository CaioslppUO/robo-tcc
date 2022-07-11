from auto_mode_calcs import * 
from mission import _Location
import math


class TestAutoMode():

    def test_should_calculate_distance_between_two_points(self):
        assert dist_two_points(0, 0, 0, 1) == 1
        assert dist_two_points(0, 0, 1, 0) == 1
        assert dist_two_points(0, 0, 1, 1) == math.sqrt(2)
        assert dist_two_points(0, 0, -1, -1) == math.sqrt(2)
    
    def test_should_convert_radians_to_degrees(self):
        assert rad_to_deg(math.pi) == 180
        assert rad_to_deg(math.pi / 2) == 90
        assert rad_to_deg(math.pi / 4) == 45

    def test_should_return_correct_turn_direction(self):
        assert need_to_correct_route(_Location(-25.0001, -56.0002, ""), -25.0000, -56.0000, 0) == "right"