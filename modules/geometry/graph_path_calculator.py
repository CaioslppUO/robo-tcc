from path_calculator import PathCalculator
from point import Point
from points import Points
from graph import plot

def test(mission_start: Point, mission_end: Point, robot_pos_1: Point, robot_pos_2: Point) -> None:
    # Mission settings
    number_of_points = 16
    correction_point_distance = 4

    # Creating path calculator
    p = PathCalculator(mission_start, mission_end)
    points, number =  p.get_points_between()

    print("Original start:      ({:10.10f}, {:10.10f})".format(p.mission_start.latitude, p.mission_start.longitude))
    print("Original end:        ({:10.10f}, {:10.10f})\n".format(p.mission_end.latitude, p.mission_end.longitude))
    print("start:               ({:10.10f}, {:10.10f})".format(p.start.latitude, p.start.longitude))
    print("end:                 ({:10.10f}, {:10.10f})\n".format(p.end.latitude, p.end.longitude))
    print("Coeficiente Angular: {:10.10f}".format(p.angular_coefficient))
    print("Coeficiente Linear:  {:10.10f}\n".format(p.linear_coefficient))
    print("Distância:           {:10.10f}".format(p.total_distance))
    print("Número de pontos:    {:10}\n".format(number))

    #print(points.get_latitudes())
    #print(points.get_longitudes())

    # Execution Simulation - Above line
    closest_point, correction_point = points.get_closest_points(robot_pos_1, correction_point_distance)
    print("robot_pos_1:         ({:10.10f}, {:10.10f})".format(robot_pos_1.latitude, robot_pos_1.longitude))
    print("closest_1:           ({:10.10f}, {:10.10f})".format(closest_point.latitude, closest_point.longitude))
    print("correction_1:        ({:10.10f}, {:10.10f})".format(correction_point.latitude, correction_point.longitude))
    
    #dif_lat, dif_lon = robot_pos_1.dif(correction_point.latitude, correction_point.longitude)
    #print("dif robotXcorrec.:   ({:10.10f}, {:10.10f})".format(dif_lat, dif_lon))
    
    #p1_closest = PathCorrection(robot_pos_1, correction_point, 1)
    #p1_correction = PathCorrection(robot_pos_1, correction_point, 1)
    p1_angular = robot_pos_1.get_angular_coefficient(closest_point)
    print("robot_1_angular_cof: ({:10.10f})".format(p1_angular))
#
    correction_direction = robot_pos_1.get_correction_direction(closest_point.latitude, closest_point.longitude, p1_angular)
    print("Correction direction pos_1: {}".format(correction_direction))
    
    plot(points, robot_pos_1, closest_point, correction_point, correction_direction)
#
    ## Execution Simulation - Under line
    #closest_point, correction_point = points.get_closest_points(robot_pos_2, correction_point_distance)
    #print("\nrobot_pos_2:         ({:10.10f}, {:10.10f})".format(robot_pos_2.latitude, robot_pos_2.longitude))
    #print("closest_2:           ({:10.10f}, {:10.10f})".format(closest_point.latitude, closest_point.longitude))
    #print("correction_2:        ({:10.10f}, {:10.10f})".format(correction_point.latitude, correction_point.longitude))
    #
    #dif_lat, dif_lon = robot_pos_2.dif(correction_point.latitude, correction_point.longitude)
    #print("dif robotXcorrec.:   ({:10.10f}, {:10.10f})".format(dif_lat, dif_lon))
    #
    #p2_closest = PathCorrection(robot_pos_2, correction_point, 1)
    #p2_correction = PathCorrection(robot_pos_2, correction_point, 1)
    #print("robot_2_angular_cof: ({:10.10f})".format(p2_correction.angular_coff))
#
    #correction_direction = robot_pos_2.get_correction_direction(closest_point.latitude, closest_point.longitude, p2_closest.angular_coff)
    #print("Correction direction pos_2: {}".format(correction_direction))
#
    #plot(points, robot_pos_2, closest_point, correction_point, correction_direction)

def test_up_right():
    # Mission
    mission_start = Point(-25.43548, -54.59701, 6)
    mission_end = Point(-25.43524, -54.59695, 6)

    robot_pos_1 = Point(0.00010, 0.00002, 6)
    robot_pos_2 = Point(0.00005, 0.00002, 6)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_up_left():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43524, -54.59709)

    robot_pos_1 = Point(0.00010, -0.00002)
    robot_pos_2 = Point(0.00002, -0.00002)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_down_right():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43555, -54.59695)

    robot_pos_1 = Point(-0.000005, 0.00001)
    robot_pos_2 = Point(-0.00002, 0.00001)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_down_left():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43555, -54.59709)

    robot_pos_1 = Point(-0.00001, -0.00002)
    robot_pos_2 = Point(-0.00003, -0.00002)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_lat_0_left():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43548, -54.59709)

    robot_pos_1 = Point(-0.00001, -0.00002)
    robot_pos_2 = Point(0.00001, -0.00002)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_lat_0_right():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43548, -54.59695)

    robot_pos_1 = Point(-0.00001, 0.00002)
    robot_pos_2 = Point(0.00001, 0.00002)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_lon_0_down():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43555, -54.59701)

    robot_pos_1 = Point(-0.00001, -0.00001)
    robot_pos_2 = Point(-0.00001, 0.00001)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

def test_lon_0_up():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43512, -54.59701)

    robot_pos_1 = Point(0.00002, -0.00002)
    robot_pos_2 = Point(0.00002, 0.00002)

    test(mission_start, mission_end, robot_pos_1, robot_pos_2)

test_up_right()
#test_up_left()
#test_down_right()
#test_down_left()
#test_lat_0_left()
#test_lat_0_right()
#test_lon_0_down()
#test_lon_0_up()