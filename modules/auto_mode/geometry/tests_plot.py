from pathlib import Path
from graph import plot
from path_calculator import PathCalculator, Point, Points

def test_1() -> None:
    # Up left
    #start = Point(-25.435348, -54.596970)
    #end = Point(-25.435324, -54.5969705)

    # Up right
    #start = Point(-25.435348, -54.596970)
    #end = Point(-25.435324, -54.596960)

    # Down left
    #start = Point(-25.435348, -54.596970)
    #end = Point(-25.435355, -54.5969705)

    # Down right
    start = Point(-25.435348, -54.596970)
    end = Point(-25.435355, -54.596960)
    path = PathCalculator(start, end, 10)

    points = path.get_points_between()
    plot(points)

def test_robot_quadrant_1() -> None:
    """
    Test the robot plot with the closest point in quadrant_1.
    """
    start = Point(-25.435348, -54.596970)
    end = Point(-25.435324, -54.596960)
    path = PathCalculator(start, end, 15)
    points = path.get_points_between()

    robot_points = Points()
    robot_points.add_point(-25.435348, -54.596970)
    robot_points.add_point(-25.435345, -54.596970)
    robot_points.add_point(-25.435348, -54.596969)
    robot_points.add_point(-25.435345, -54.596968)
    robot_points.add_point(-25.435338, -54.596967)
    robot_points.add_point(-25.435337, -54.596966)
    robot_points.add_point(-25.435337, -54.596965)
    robot_points.add_point(-25.435335, -54.596964)
    robot_points.add_point(-25.435330, -54.596963)
    robot_points.add_point(-25.435330, -54.596962)
    robot_points.add_point(-25.435326, -54.596961)
    robot_points.add_point(-25.435324, -54.596960)


    for r_point in robot_points.get_points():
        _a, _b = points.get_closest_points(r_point, 3)
        closest_point = Point(_a.latitude, _a.longitude)
        correction_point = Point(_b.latitude, _b.longitude)

        correction_direction = r_point.get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        plot(points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=path.get_mission_quadrant())

def test_robot_quadrant_2() -> None:
    """
    Test the robot plot with the closest point in quadrant_1.
    """
    start = Point(-25.435348, -54.596970)
    end = Point(-25.435355, -54.596960)
    path = PathCalculator(start, end, 15)
    points = path.get_points_between()

    robot_points = Points()
    robot_points.add_point(-25.435348, -54.596969)
    robot_points.add_point(-25.435349, -54.596970)
    robot_points.add_point(-25.435349, -54.596969)
    robot_points.add_point(-25.435351, -54.596967)
    robot_points.add_point(-25.435351, -54.596966)
    robot_points.add_point(-25.435351, -54.596965)
    robot_points.add_point(-25.435352, -54.596964)
    robot_points.add_point(-25.435353, -54.596963)
    robot_points.add_point(-25.435354, -54.596962)
    robot_points.add_point(-25.435354, -54.596961)
    robot_points.add_point(-25.435355, -54.596960)


    for r_point in robot_points.get_points():
        _a, _b = points.get_closest_points(r_point, 3)
        closest_point = Point(_a.latitude, _a.longitude)
        correction_point = Point(_b.latitude, _b.longitude)

        correction_direction = r_point.get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        plot(points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=path.get_mission_quadrant())

def test_robot_quadrant_3() -> None:
    """
    Test the robot plot with the closest point in quadrant_1.
    """
    start = Point(-25.435348, -54.596970)
    end = Point(-25.435355, -54.5969705)
    path = PathCalculator(start, end, 15)
    points = path.get_points_between()

    robot_points = Points()
    robot_points.add_point(-25.435348, -54.596970)
    robot_points.add_point(-25.435349, -54.596970)
    robot_points.add_point(-25.435348, -54.5969701)
    robot_points.add_point(-25.435349, -54.5969701)
    robot_points.add_point(-25.435351, -54.5969702)
    robot_points.add_point(-25.435352, -54.5969703)
    robot_points.add_point(-25.435353, -54.5969704)
    robot_points.add_point(-25.435354, -54.5969704)
    robot_points.add_point(-25.435355, -54.5969705)

    for r_point in robot_points.get_points():
        _a, _b = points.get_closest_points(r_point, 3)
        closest_point = Point(_a.latitude, _a.longitude)
        correction_point = Point(_b.latitude, _b.longitude)

        correction_direction = r_point.get_correction_direction(start, path.get_angular_coefficient(), path.get_mission_quadrant())
        plot(points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=path.get_mission_quadrant())

#test_1()

#test_robot_quadrant_1()
#test_robot_quadrant_2()
test_robot_quadrant_3()