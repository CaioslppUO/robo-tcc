from points import Points, Point

class PathCalculator:
    def __init__(self, mission_start: Point, mission_end: Point, number_of_points: int = 8, decimals: int = 6) -> None:
        # Loading mission data
        self.mission_start = mission_start
        self.mission_end = mission_end
        self.__number_of_points = number_of_points
        self.__decimals = decimals

        # Moving start to (0, 0) and adjusting end points.
        self.start = Point(0, 0, self.__decimals)
        self.end = Point(
            mission_end.latitude - mission_start.latitude,
            mission_end.longitude - mission_start.longitude,
            self.__decimals
        )

        # Use the smaller value to set the increment between each point
        if(self.end.latitude > self.end.longitude and self.end.longitude != 0):
            self.points_interval = round(self.end.longitude / self.__number_of_points, self.__decimals)
        elif(self.end.latitude != 0):
            self.points_interval = round(self.end.latitude / self.__number_of_points, self.__decimals)
        else:
            self.points_interval = round(self.end.longitude / self.__number_of_points, self.__decimals)

        # Line calculations
        self.angular_coefficient: float = self.start.get_angular_coefficient(self.end)
        self.linear_coefficient: float = self.start.get_linear_coefficient(self.angular_coefficient)
        self.total_distance: float = self.start.get_distance(self.end)

    def get_y_line_equation(self, lon: float) -> float:
        """
        Calculate Y value with formula: y = m * x + n.
        """
        return round(self.angular_coefficient * lon + self.linear_coefficient, self.__decimals)

    def get_x_line_equation(self, lat: float,) -> float:
        """
        Calculate Y value with formula: y = m * x + n.
        """
        return round((lat - self.linear_coefficient) / self.angular_coefficient, self.__decimals)

    def get_points_between(self) -> "tuple[Points, int]":
        """
        Return n points between mission_start and mission_end.
        """
        points = Points(self.__decimals)

        longitude = 0
        for _ in range(0, self.__number_of_points):
            longitude = round(longitude, self.__decimals)
            latitude = self.get_y_line_equation(longitude)

            if(self.angular_coefficient != 0):
                if(abs(latitude) > abs(self.end.latitude) or abs(longitude) > abs(self.end.longitude)):
                    print(abs(latitude) > abs(self.end.latitude))
                    break
                points.add_point(latitude, longitude)
            elif(self.end.longitude == 0):
                if(abs(longitude) > abs(self.end.latitude)):
                    break
                points.add_point(longitude, 0)
            else:
                if(abs(longitude) > abs(self.end.longitude)):
                    break
                points.add_point(0, longitude)

            if(self.angular_coefficient < 0 and self.end.longitude > 0):
                longitude -= self.points_interval
            else:
                longitude += self.points_interval

        points.add_point(round(self.end.latitude, self.__decimals), round(self.end.longitude, self.__decimals))

        return points, len(points.get_latitudes())