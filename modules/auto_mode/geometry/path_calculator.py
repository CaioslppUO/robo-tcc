from points import Points, Point

class PathCalculator:
    def __init__(self, decimals: int, start: Point, end: Point, number_of_points: int) -> None:
        self.__decimals = decimals
        self.__start = Point(start.latitude, start.longitude, decimals)
        self.__end = Point(end.latitude, end.longitude, decimals)
        self.__number_of_points = number_of_points

        self.__angular_coefficient = self.__start.get_angular_coefficient(self.__end)
        self.__linear_coefficient = self.__start.get_linear_coefficient(self.__angular_coefficient)

        self.__interval = abs(self.__start.get_difference(self.__end).longitude / (number_of_points - 1))

    def get_interval(self) -> float:
        return round(self.__interval, self.__decimals)

    def get_angular_coefficient(self) -> float:
        return round(self.__angular_coefficient, self.__decimals)

    def get_linear_coefficient(self) -> float:
        return round(self.__linear_coefficient, self.__decimals)
 
    def get_y_line_equation(self, longitude: float) -> float:
        """
        Get y value from line equation.
        """
        return round((self.__angular_coefficient * longitude + self.__linear_coefficient), self.__decimals)

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
                print("subtraiu")
                longitude -= self.__interval

        points.add_point(self.__end.latitude, self.__end.longitude)

        return points