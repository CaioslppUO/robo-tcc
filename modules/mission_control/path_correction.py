from point import Points, Point
from auto_mode_calcs import dist_two_points
from graph import plot
import math

class PathCorrection:
    def __init__(self, mission_start: Point, mission_end: Point, number_of_points: int) -> None:
        self.mission_start = mission_start
        self.mission_end = mission_end
        self.number_of_points = number_of_points

        self.start = Point(0, 0)
        self.end = Point(
            mission_end.latitude - mission_start.latitude,
            mission_end.longitude - mission_start.longitude
        )
        
        # Use the smaller value to set the increment between each point
        if(self.end.latitude > self.end.longitude and self.end.longitude != 0):
            self.points_interval = self.end.longitude / number_of_points
        elif(self.end.latitude != 0):
            self.points_interval = self.end.latitude / number_of_points
        else:
            self.points_interval = self.end.longitude / number_of_points
        # Update the scale due to how many points were requested
        self.scale = len(str(self.points_interval))

        self.angular_coff: float = round(self.__calculate_angular_coefficient(), self.scale)
        self.linear_coff: float = round(self.__calculate_linear_coefficient(), self.scale)

        self.total_distance: float = round(dist_two_points(self.start.latitude, self.start.longitude, self.end.latitude, self.end.longitude), self.scale)

    def __calculate_angular_coefficient(self) -> float:
        """
        Calculate angular coefficient.
        """
        if(self.end.longitude - self.start.longitude == 0):
            return 0
        return (self.end.latitude - self.start.latitude) / (self.end.longitude - self.start.longitude)

    def __calculate_linear_coefficient(self) -> float:
        """
        Calculate linear coefficient.
        """
        if(self.start.latitude == 0 and self.start.longitude == 0):
            return 0
        return self.start.latitude / (self.start.longitude * self.angular_coff)

    def get_y_line_equation(self, lon: float) -> float:
        """
        Calculate Y value with formula: y = m * x + n.
        """
        return self.angular_coff * lon + self.linear_coff

    def get_x_line_equation(self, lat: float) -> float:
        """
        Calculate Y value with formula: y = m * x + n.
        """
        return (lat - self.linear_coff) / self.angular_coff

    def get_points_between(self) -> "tuple[Points, int]":
        """
        Return distance / points_interval points between mission_start and mission_end.
        """
        points = Points()
        
        #if(self.end.longitude > self.end.latitude):
        longitude = 0
        for _ in range(0, self.number_of_points - 1):
            longitude = round(longitude, self.scale)
            latitude = self.get_y_line_equation(longitude)

            if(self.angular_coff != 0):
                if(abs(round(latitude, self.scale)) > abs(self.end.latitude) or abs(longitude) > abs(self.end.longitude)):
                    break
                points.add_point(round(latitude, self.scale), longitude)
            elif(self.end.longitude == 0):
                if(abs(longitude) > abs(self.end.latitude)):
                    break
                points.add_point(longitude, 0)
            else:
                if(abs(longitude) > abs(self.end.longitude)):
                    break
                points.add_point(0, longitude)

            if(self.angular_coff < 0 and self.end.longitude > 0):
                longitude -= self.points_interval
            else:
                longitude += self.points_interval
                    
        points.add_point(round(self.end.latitude, self.scale), round(self.end.longitude, self.scale))

        return points, len(points.get_latitudes())

def test(mission_start: Point, mission_end: Point, robot_pos_1: Point, robot_pos_2: Point) -> None:
    number_of_points = 16
    correction_point_distance = 4

    p = PathCorrection(mission_start, mission_end, number_of_points)
    points, number =  p.get_points_between()

    print("Original start:      ({:10.10f}, {:10.10f})".format(p.mission_start.latitude, p.mission_start.longitude))
    print("Original end:        ({:10.10f}, {:10.10f})\n".format(p.mission_end.latitude, p.mission_end.longitude))
    print("start:               ({:10.10f}, {:10.10f})".format(p.start.latitude, p.start.longitude))
    print("end:                 ({:10.10f}, {:10.10f})\n".format(p.end.latitude, p.end.longitude))
    print("Coeficiente Angular: {:10.10f}".format(p.angular_coff))
    print("Coeficiente Linear:  {:10.10f}\n".format(p.linear_coff))
    print("Distância:           {:10.10f}".format(p.total_distance))
    print("Número de pontos:    {:10}".format(number))

    #print(points.get_latitudes())
    #print(points.get_longitudes())

    # Execution Simulation - Above line
    closest_point, correction_point = points.get_closest_points(robot_pos_1, correction_point_distance)
    plot(points, robot_pos_1, closest_point, correction_point)

    # Execution Simulation - Under line
    closest_point, correction_point = points.get_closest_points(robot_pos_2, correction_point_distance)
    plot(points, robot_pos_2, closest_point, correction_point)

def test_up_right():
    # Mission
    mission_start = Point(-25.43548, -54.59701)
    mission_end = Point(-25.43524, -54.59695)

    robot_pos_1 = Point(0.00010, 0.00002)
    robot_pos_2 = Point(0.00005, 0.00002)

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
test_up_left()
test_down_right()
test_down_left()
test_lat_0_left()
test_lat_0_right()
test_lon_0_down()
test_lon_0_up()