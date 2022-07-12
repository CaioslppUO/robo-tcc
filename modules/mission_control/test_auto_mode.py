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

        ## Near 0
        assert need_to_correct_route(_Location(-24.9200, -55.5000, ""), -25.0000, -56.0000, 0) == "forward"
        assert need_to_correct_route(_Location(-24.9550, -55.5000, ""), -25.0000, -56.0000, 356) == "forward"
        ## Near 45
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 36) == "forward"
        assert need_to_correct_route(_Location(-24.5000, -55.5000, ""), -25.0000, -56.0000, 55) == "forward"
        ## Near 90
        assert need_to_correct_route(_Location(-24.5000, -55.9150, ""), -25.0000, -56.0000, 90) == "forward"
        assert need_to_correct_route(_Location(-24.5000, -56.0000, ""), -25.0000, -56.0000, 100) == "forward"

        # Up left quadrant
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 0) == "left"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 90) == "left"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 180) == "right"
        assert need_to_correct_route(_Location(-26.0000, -55.5000, ""), -25.0000, -56.0000, 270) == "right"

        ## Near 0
        assert need_to_correct_route(_Location(-25.0400, -55.5000, ""), -25.0000, -56.0000, 5) == "forward"
        assert need_to_correct_route(_Location(-25.0400, -55.5000, ""), -25.0000, -56.0000, 346) == "forward"
        ## Near 315
        assert need_to_correct_route(_Location(-25.5000, -55.5000, ""), -25.0000, -56.0000, 305) == "forward"
        assert need_to_correct_route(_Location(-25.5000, -55.5000, ""), -25.0000, -56.0000, 325) == "forward"
        ## Near 270
        assert need_to_correct_route(_Location(-31.0000, -55.5000, ""), -25.0000, -56.0000, 265) == "forward"
        assert need_to_correct_route(_Location(-31.0000, -55.5000, ""), -25.0000, -56.0000, 284) == "forward"

        # Down right quadrant
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 0) == "right"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 90) == "right"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 190) == "left"
        assert need_to_correct_route(_Location(-24.5000, -57.0000, ""), -25.0000, -56.0000, 270) == "left"

        ## Near 90
        assert need_to_correct_route(_Location(-14.0000, -57.0000, ""), -25.0000, -56.0000, 86) == "forward"
        assert need_to_correct_route(_Location(-14.0000, -57.0000, ""), -25.0000, -56.0000, 105) == "forward"
        ## Near 135
        assert need_to_correct_route(_Location(-24.0000, -57.0000, ""), -25.0000, -56.0000, 125) == "forward"
        assert need_to_correct_route(_Location(-24.0000, -57.0000, ""), -25.0000, -56.0000, 145) == "forward"
        ## Near 180
        assert need_to_correct_route(_Location(-25.0000, -57.0000, ""), -25.0000, -56.0000, 170) == "forward"
        assert need_to_correct_route(_Location(-25.0000, -57.0000, ""), -25.0000, -56.0000, 190) == "forward"

        # Down left quadrant
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 0) == "left"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 90) == "right"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 180) == "right"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 270) == "left"

        ## Near 180
        assert need_to_correct_route(_Location(-26.0000, -90.0000, ""), -25.0000, -56.0000, 172) == "forward"
        assert need_to_correct_route(_Location(-26.0000, -90.0000, ""), -25.0000, -56.0000, 191) == "forward"
        ## Near 225
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 215) == "forward"
        assert need_to_correct_route(_Location(-26.0000, -57.0000, ""), -25.0000, -56.0000, 235) == "forward"
        ## Near 270
        assert need_to_correct_route(_Location(-100.0000, -57.0000, ""), -25.0000, -56.0000, 260) == "forward"
        assert need_to_correct_route(_Location(-100.0000, -57.0000, ""), -25.0000, -56.0000, 279) == "forward"

        # Erros
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), 0, -56.0000, 0) == "error"
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), -25.0000, 0, 0) == "error"
        assert need_to_correct_route(_Location(-24.9999, -55.9999, ""), 0, 0, 0) == "error"