from __future__ import annotations
from math import sqrt

class Point:
    def __init__(self, latitude: float, longitude: float, decimals: int = 7, id: int = 0) -> None:
        self.__decimals = 7
        self.longitude = longitude
        self.latitude = latitude
        self.id = id

    def get_point(self) -> "tuple[float, float]":
        """
        Return the (latitude, longitude).
        """
        return (self.latitude, self.longitude)

    def equal(self, other: Point) -> bool:
        """
        Return ture if other is the same as self.
        """
        return self.latitude == other.latitude and self.longitude == other.longitude

    def difference(self, other: Point) -> Point:
        """
        Return a point with the difference (other.lat - self.lat, other.lon - self.lon).
        """ 
        return Point(other.latitude - self.latitude, other.longitude - self.longitude)

    def distance(self, other: Point) -> float:
        """
        Return the distance between two points.
        """
        a1 = (other.latitude - self.latitude) ** 2
        a2 = (other.longitude - self.longitude) ** 2
        return round(sqrt(a1 + a2), self.__decimals)