from points import Points, Point

class PathCalculator:
    def __init__(self, start: Point, end: Point, number_of_points: int, decimals: int = 7) -> None:
        self.__decimals = decimals
        self.__start = Point(start.latitude, start.longitude, decimals)
        self.__end = Point(end.latitude, end.longitude, decimals)
        self.__number_of_points = number_of_points

        self.__angular_coefficient = self.__start.get_angular_coefficient(self.__end)
        self.__linear_coefficient = self.__start.get_linear_coefficient(self.__angular_coefficient)

        self.__interval = abs(self.__start.get_difference(self.__end).longitude / (number_of_points - 1))
        self.__quadrant = self.get_mission_quadrant()

    def get_interval(self) -> float:
        """
        Return the interval between each point in the x axis (longitude).
        """
        return round(self.__interval, self.__decimals)

    def get_angular_coefficient(self) -> float:
        """
        Return the angular coefficient.
        """
        return round(self.__angular_coefficient, self.__decimals)

    def get_linear_coefficient(self) -> float:
        """
        Return the linear coefficient.
        """
        return round(self.__linear_coefficient, self.__decimals)
 
    def get_y_line_equation(self, longitude: float) -> float:
        """
        Get y value from line equation.
        """
        return round((self.__angular_coefficient * longitude + self.__linear_coefficient), self.__decimals)

    def get_mission_quadrant(self) -> int:
        """
        Return in which quadrant is the mission.
        """
        if(self.__end.latitude > self.__start.latitude and self.__end.longitude > self.__start.longitude):
            return 1
        elif(self.__end.latitude > self.__start.latitude and self.__end.longitude < self.__start.longitude):
            return 4
        elif(self.__end.latitude < self.__start.latitude and self.__end.longitude > self.__start.longitude):
            return 2
        return 3

    def get_points_between(self) -> Points:
        """
        Return number_of_points between start and end.
        """
        points = Points(self.__decimals)

        longitude = self.__start.longitude
        for i in range(0, self.__number_of_points - 1):
            latitude = self.get_y_line_equation(longitude)
            points.add_point(latitude, longitude)

            if(self.__end.longitude > self.__start.longitude):
                longitude += self.__interval
            else:
                longitude -= self.__interval

        points.add_point(self.__end.latitude, self.__end.longitude)

        return points