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
        # Up right quadrant
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 0) == "right"
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 90) == "left"
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 180) == "left"
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 270) == "right"

        # Up left quadrant
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 0) == "left"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 90) == "left"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 180) == "right"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 270) == "right"

        # Down right quadrant
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 0) == "right"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 90) == "right"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 190) == "left"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 270) == "left"

        # Down left quadrant
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 0) == "left"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 90) == "right"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 180) == "right"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 270) == "left"

        # Erros
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), 0, -56.0000, 0) == "error"
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), -25.0000, 0, 0) == "error"
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), 0, 0, 0) == "error"