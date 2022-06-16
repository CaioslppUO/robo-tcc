from object_detector import ObjectDetector

class TestDetectObject:

    def test_should_detect_object_in_front(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_distance(45) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == True

    def test_should_detect_object_in_front_2(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_distance(44) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == True
        
    def test_should_not_detect_object_in_front(self):
        secure_distance = 45 # CM
        obj_dt: ObjectDetector = ObjectDetector(secure_distance)
        obj_dt.set_distance(60) # Setting an object in front in a 45cm distance
        assert obj_dt.has_object_in_front() == False