from __future__ import annotations
import math

class Point:
    def __init__(self, latitude: float, longitude: float, decimals: int = 7, id: int = 0) -> None:
        self.__decimals = decimals
        self.latitude = latitude
        self.longitude = longitude
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
            return float("inf")
        return round((end_point.latitude - self.latitude) / (end_point.longitude - self.longitude), self.__decimals)

    def get_linear_coefficient(self, angular_coefficient: float) -> float:
        return round(self.latitude - self.longitude * angular_coefficient, self.__decimals)

    def get_correction_direction(self, start_point: Point, initial_angular_coefficient: float, mission_quadrant: int, closest_point: Point) -> str:
        """
        Return the correction direction based on the quadrant and angular_coefficient.
        """
        angular_coefficient = self.get_angular_coefficient(start_point)
        
        quadrant_1 = mission_quadrant == 1
        quadrant_2 = mission_quadrant == 2
        quadrant_3 = mission_quadrant == 3
        quadrant_4 = mission_quadrant == 4
        bigger_angular = angular_coefficient != 0 and angular_coefficient > initial_angular_coefficient
        lat_diff = self.latitude - closest_point.latitude
        lon_diff = self.longitude - closest_point.longitude

        if(angular_coefficient == float("inf")): # Only axis y
            if(lon_diff == 0): # Is in the point
                return "forward"
            elif(quadrant_1):
                if(lon_diff > 0): # Is on the left of the point
                    return "right"
                else: # Is on the right of the point
                    return "left"
        elif(angular_coefficient == 0): # Only axis x
            pass
        

    #def get_correction_direction(self, latitude: float, longitude: float, angular_coefficient: float) -> str:
    #    if(angular_coefficient > 0 and latitude > self.latitude and longitude > self.longitude and latitude > 0 and longitude > 0):
    #        return "right"
    #    elif(angular_coefficient > 0 and latitude < self.latitude and longitude < self.longitude and latitude > 0 and longitude > 0):
    #        return "left"
    #    elif(angular_coefficient < 0 and latitude < self.latitude and longitude < self.longitude and latitude > 0 and longitude < 0):
    #        return "left"
    #    elif(angular_coefficient < 0 and latitude > self.latitude and longitude > self.longitude and latitude > 0 and longitude < 0):
    #        return "right"
    #    elif(angular_coefficient < 0 and latitude < self.latitude and longitude < self.longitude and latitude < 0 and longitude > 0):
    #        return "right"
    #    elif(angular_coefficient < 0 and latitude > self.latitude and longitude > self.longitude and latitude < 0 and longitude > 0):
    #        return "left"
    #    elif(angular_coefficient > 0 and latitude < self.latitude and longitude > self.longitude and latitude < 0 and longitude < 0):
    #        return "left"
    #    elif(angular_coefficient > 0 and latitude > self.latitude and longitude < self.longitude and latitude < 0 and longitude < 0):
    #        return "right"
    #    elif(latitude == 0 and self.latitude < 0 and longitude < 0):
    #        return "right"
    #    elif(latitude == 0 and self.latitude > 0 and longitude < 0):
    #        return "left"
    #    elif(latitude == 0 and self.latitude < 0 and longitude > 0):
    #        return "left"
    #    elif(latitude == 0 and self.latitude > 0 and longitude > 0):
    #        return "right"
    #    elif(longitude == 0 and self.longitude < 0 and latitude < 0):
    #        return "left"
    #    elif(longitude == 0 and self.longitude > 0 and latitude < 0):
    #        return "right"
    #    elif(longitude == 0 and self.longitude < 0 and latitude > 0):
    #        return "right"
    #    elif(longitude == 0 and self.longitude > 0 and latitude > 0):
    #        return "left"
    #    return "forward"