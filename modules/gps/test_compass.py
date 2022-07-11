from compass_2 import get_angle, calculate_compass

class TestCompass:

    def test_should_return_the_quadrant(self):
        _, quadrant, _ = get_angle(-1, -1, 1, 1)
        assert quadrant == 1
        _, quadrant, _ = get_angle(1, 1, -1, -1)
        assert quadrant == 3
        _, quadrant, _ = get_angle(-1, 1, 1, -1)
        assert quadrant == 2
        _, quadrant, _ = get_angle(1, -1, -1, 1)
        assert quadrant == 4
        _, quadrant, _ = get_angle(1, -1, -1, 1)
        assert quadrant == 4

    def test_should_return_the_correct_angles(self):
        angle, _, _ = get_angle(0, 1, 10, 1)
        assert angle == 0
        angle, _, compass = get_angle(0, -1, 0, 10)
        assert angle == 180
        angle, _, compass = get_angle(0, 1, 0, -10)
        assert angle == 180

    def test_should_return_correct_compass(self):
        assert calculate_compass(20, 1) == 70
        assert calculate_compass(45, 1) == 45
        assert calculate_compass(30, 2) == 120
        assert calculate_compass(43, 2) == 133
        assert calculate_compass(27, 3) == 207
        assert calculate_compass(68, 3) == 248
        assert calculate_compass(75, 4) == 345

    def test_simulate_robot(self):
        p1_x, p1_y = -25.437556, -54.595835
        #p2_x, p2_y = -25.437295, -54.595836
        p2_x, p2_y = -25.437295, -54.595835
        p3_x, p3_y = -25.437550, -54.596213
        p4_x, p4_y = -25.437612, -54.595972
        print(get_angle(p3_x, p3_y, p4_x, p4_y))
        assert 1 == 12