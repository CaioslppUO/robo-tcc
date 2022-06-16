from control_movement import ControlMovement

class TestControlMovement:
    
    def test_should_prevent_movement(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_distance(44)
        assert ctr_mv.can_move() == False

    def test_should_prevent_movement_4(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_distance(45)
        assert ctr_mv.can_move() == False

    def test_should_allow_movement(self):
        secure_distance = 45 #cm
        ctr_mv = ControlMovement(secure_distance)
        ctr_mv.object_detector.set_distance(46)
        assert ctr_mv.can_move() == True