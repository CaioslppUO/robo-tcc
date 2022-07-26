from math import sqrt

class Point:
    def __init__(self, latitude: float, longitude: float, decimals: int = 7, id: int = 0) -> None:
        self.__decimals = 7
        self.longitude = longitude
        self.latitude = latitude
        self.id = id

    def set_point(self, latitude: float, longitude: float) -> None:
        """
        Set the point to the given latitude and longitude.
        """
        self.latitude = latitude
        self.longitude = longitude

    def get_point(self) -> "tuple[float, float]":
        """
        Return the (latitude, longitude).
        """
        return (self.latitude, self.longitude)

    def get_latitude(self) -> float:
        """
        Return the latitude.
        """
        return self.latitude
    
    def get_longitude(self) -> float:
        """
        Return the longitude.
        """
        return self.longitude

    def equal(self, other_lat: float, other_long: float) -> bool:
        """
        Return ture if other is the same as self.
        """
        return self.latitude == other_lat and self.longitude == other_long

    def difference(self, other_lat: float, other_lon: float) -> "tuple[float, float]":
        """
        Return a point with the difference (other.lat - self.lat, other.lon - self.lon).
        """ 
        return (other_lat - self.latitude, other_lon - self.longitude)

    def distance(self,  other_lat: float, other_lon: float) -> float:
        """
        Return the distance between two points.
        """
        a1 = (other_lat - self.latitude) ** 2
        a2 = (other_lon - self.longitude) ** 2
        return round(sqrt(a1 + a2), self.__decimals)
    
    def is_zero(self) -> bool:
        """
        Return True if a point is (0,0).
        """
        return self.latitude == 0 and self.longitude == 0