from object_detector import ObjectDetector

class TestDetectObject:

    def test_should_detect_object_in_front(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_front(45) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == True

    def test_should_detect_object_in_left(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_left(45) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_left() == True

    def test_should_detect_object_in_right(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_right(45) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_right() == True

    def test_should_detect_object_in_front_2(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_front(44) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == True

    def test_should_detect_object_in_left_2(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_left(44) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_left() == True

    def test_should_detect_object_in_right_2(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_right(44) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_right() == True

    def test_should_not_detect_object_in_front(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_front(60) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == False

    def test_should_not_detect_object_in_left(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_left(60) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_left() == False

    def test_should_not_detect_object_in_right(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_right(60) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_right() == False