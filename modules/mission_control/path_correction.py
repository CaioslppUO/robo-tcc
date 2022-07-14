from point import Points, Point
from auto_mode_calcs import dist_two_points
from graph import plot
import math

class PathCorrection:
    def __init__(self, points_interval: float, mission_start: Point, mission_end: Point) -> None:
        approx_value = 5
        self.points_interval: float = round(points_interval, approx_value)
        self.mission_start = mission_start
        self.mission_end = mission_end

        self.start = Point(0, 0)
        self.end = Point(
            mission_end.latitude - mission_start.latitude,
            mission_end.longitude - mission_start.longitude
        )

        self.angular_coff: float = round(self.__calculate_angular_coefficient(), approx_value)
        self.linear_coff: float = round(self.__calculate_linear_coefficient(), approx_value)

        self.total_distance: float = round(dist_two_points(self.start.latitude, self.start.longitude, self.end.latitude, self.end.longitude), approx_value)
        self.number_of_points: int = 1 + int(math.floor(self.total_distance / self.points_interval))

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

    def line_equation(self, lon: float) -> float:
        """
        Calculate Y value with formula: y = m * x + n.
        """
        return self.angular_coff * lon + self.linear_coff

    def get_points_between(self) -> Points:
        """
        Return distance / points_interval points between mission_start and mission_end.
        """
        points = Points()
        
        longitude = 0
        for _ in range(0, self.number_of_points - 1):
            longitude = round(longitude, 5)
            latitude = self.line_equation(longitude)

            if(self.angular_coff != 0):
                points.add_point(round(latitude, 5), longitude)
            elif(self.end.longitude == 0):
                points.add_point(longitude, 0)
            else:
                points.add_point(0, longitude)

            if((self.end.latitude > 0 and self.end.longitude < 0) or (self.end.latitude < 0 and self.end.longitude < 0)):
                longitude -= self.points_interval
            else:
                longitude += self.points_interval

        points.add_point(self.end.latitude, self.end.longitude)

        return points

def test() -> None:
    points_interval = 0.3
    mission_start = Point(2, 2)
    mission_end = Point(3, 1)

    p = PathCorrection(points_interval, mission_start, mission_end)

    print("Original start: ({:10.5f}, {:10.5f})".format(p.mission_start.latitude, p.mission_start.longitude))
    print("Original end: ({:10.5f}, {:10.5f})".format(p.mission_end.latitude, p.mission_end.longitude))
    print("start: ({:10.5f}, {:10.5f})".format(p.start.latitude, p.start.longitude))
    print("end: ({:10.5f}, {:10.5f})".format(p.end.latitude, p.end.longitude))
    print("Coeficiente Angular: {:10.5f}".format(p.angular_coff))
    print("Coeficiente Linear: {:10.5f}".format(p.linear_coff))
    print("Distância: {:10.5f}".format(p.total_distance))
    print("Total de pontos: {:10}".format(p.number_of_points))
    print(p.get_points_between().get_latitudes())
    print(p.get_points_between().get_longitudes())
    plot(p.get_points_between())

test()