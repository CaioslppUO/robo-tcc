from __future__ import annotations
import math

class Point:
    def __init__(self, latitude: float, longitude: float, decimals: float, id: int = 0) -> None:
        self.__decimals = decimals
        self.latitude = round(latitude, self.__decimals)
        self.longitude = round(longitude, self.__decimals)
        self.id = id

    def get_difference(self, end_point: Point) -> Point:
        """
        Calculate the difference between self and point.
        """
        return Point(end_point.latitude - self.latitude, end_point.longitude - self.longitude, self.__decimals)

    def get_distance(self, end_point: Point) -> float:
        """
        Calculate the distance between self and point.
        """
        f1 = (end_point.latitude - self.latitude) ** 2
        f2 = (end_point.longitude - self.longitude) ** 2
        return round(math.sqrt(f1 + f2), self.__decimals)

    def get_angular_coefficient(self, end_point: Point) -> float:
        """
        Calculate the slope from the self to the point.
        """
        if(end_point.longitude - self.longitude == 0):
            return round(0, self.__decimals)
        return round((end_point.latitude - self.latitude) / (end_point.longitude - self.longitude), self.__decimals)

    def get_linear_coefficient(self, angular_coefficient: float) -> float:
        if(self.latitude == 0 and self.longitude == 0):
            return 0
        return round(self.latitude / (self.longitude * angular_coefficient), self.__decimals)

    def get_correction_direction(self, latitude: float, longitude: float, angular_coefficient: float) -> str:
        if(angular_coefficient > 0 and latitude > self.latitude and longitude > self.longitude and latitude > 0 and longitude > 0):
            return "right"
        elif(angular_coefficient > 0 and latitude < self.latitude and longitude < self.longitude and latitude > 0 and longitude > 0):
            return "left"
        elif(angular_coefficient < 0 and latitude < self.latitude and longitude < self.longitude and latitude > 0 and longitude < 0):
            return "left"
        elif(angular_coefficient < 0 and latitude > self.latitude and longitude > self.longitude and latitude > 0 and longitude < 0):
            return "right"
        elif(angular_coefficient < 0 and latitude < self.latitude and longitude < self.longitude and latitude < 0 and longitude > 0):
            return "right"
        elif(angular_coefficient < 0 and latitude > self.latitude and longitude > self.longitude and latitude < 0 and longitude > 0):
            return "left"
        elif(angular_coefficient > 0 and latitude < self.latitude and longitude > self.longitude and latitude < 0 and longitude < 0):
            return "left"
        elif(angular_coefficient > 0 and latitude > self.latitude and longitude < self.longitude and latitude < 0 and longitude < 0):
            return "right"
        elif(latitude == 0 and self.latitude < 0 and longitude < 0):
            return "right"
        elif(latitude == 0 and self.latitude > 0 and longitude < 0):
            return "left"
        elif(latitude == 0 and self.latitude < 0 and longitude > 0):
            return "left"
        elif(latitude == 0 and self.latitude > 0 and longitude > 0):
            return "right"
        elif(longitude == 0 and self.longitude < 0 and latitude < 0):
            return "left"
        elif(longitude == 0 and self.longitude > 0 and latitude < 0):
            return "right"
        elif(longitude == 0 and self.longitude < 0 and latitude > 0):
            return "right"
        elif(longitude == 0 and self.longitude > 0 and latitude > 0):
            return "left"
        return "forward"