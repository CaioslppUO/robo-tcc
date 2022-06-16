from control_movement import ControlMovement

class TestControlMovement:
    
    def test_should_prevent_movement(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_front(44)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_2(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_left(44)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_3(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_right(44)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_4(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_front(45)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_5(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_left(45)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_6(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_right(45)
        assert ctr_mv.can_move() == False

    def test_should_allow_movement(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_front(46)
        assert ctr_mv.can_move() == True

    def test_should_allow_movement_2(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_left(46)
        assert ctr_mv.can_move() == True

    def test_should_allow_movement_3(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_right(46)
        assert ctr_mv.can_move() == True