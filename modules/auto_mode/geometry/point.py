from __future__ import annotations
import math

class Point:
    def __init__(self, latitude: float, longitude: float, decimals: int = 7, id: int = 0) -> None:
        self.__decimals = decimals
        self.latitude = latitude
        self.longitude = longitude
        self.id = id

    def equals(self, other: Point, dist: float = 0.0000015) -> bool:
        """
        Return True if both points are the same.
        """
        return self.get_distance(other) <= dist, self.get_distance(other)

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
        bigger_angular = angular_coefficient > initial_angular_coefficient

        lat_1 = start_point.latitude
        lon_1 = start_point.longitude
        lat_2 = self.latitude
        lon_2 = self.longitude
        
        if(angular_coefficient == float("inf")): # Y axis movement
            if(lat_2 == lat_1):
                return "forward"
            if((quadrant_1 or quadrant_2) and lat_2 > lat_1):
                return "right"
            elif((quadrant_1 or quadrant_2) and lat_2 < lat_1):
                return "left"
            elif((quadrant_3 or quadrant_4) and lat_2 > lat_1):
                return "left"
            elif((quadrant_3 or quadrant_4) and lat_2 < lat_1):
                return "right"
            return "forward"
        elif(angular_coefficient == 0): # X axis movement
            if(lon_2 == lon_1):
                return "forward"
            if((quadrant_1 or quadrant_4) and lon_2 > lon_1):
                return "left"
            elif((quadrant_1 or quadrant_4) and lon_2 < lon_1):
                return "right"
            elif((quadrant_2 or quadrant_3) and lon_2 > lon_1):
                return "right"
            elif((quadrant_2 or quadrant_3) and lon_2 < lon_1):
                return "left"
            return "forward"
        else: # Both X and Y axis movement
            if(angular_coefficient == initial_angular_coefficient):
                print("bla")
                return "forward"
            if((quadrant_1 or quadrant_3) and bigger_angular):
                return "right"
            elif((quadrant_1 or quadrant_3) and not bigger_angular):
                return "left"
            elif((quadrant_2 or quadrant_4) and not bigger_angular):
                return "left"
            elif((quadrant_2 or quadrant_4) and bigger_angular):
                return "right"
            return "forward"