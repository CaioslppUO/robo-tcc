from point import Point

class Points:
    def __init__(self, decimals: int = 7) -> None:
        self.__points: "list[Point]" = []
        self.__decimals = decimals

    def add_point(self, latitude: float, longitude: float) -> None:
        """
        Add a new point to the points list.
        """
        self.__points.append(Point(latitude, longitude, self.__decimals, len(self.__points)))

    def get_latitudes(self) -> "tuple[float]":
        """
        Return all latitudes.
        """
        lat: "list[float]" = []
        for point in self.__points:
            lat.append(point.latitude)
        return lat
    
    def get_longitudes(self) -> "tuple[float]":
        """
        Return all longitudes.
        """
        lon: "list[float]" = []
        for point in self.__points:
            lon.append(point.longitude)
        return lon

    def get_points(self) -> "list[Point]":
        """
        Return the list of points.
        """
        return self.__points

    def get_point(self, index: int) -> Point:
        """
        Return a point.
        """
        return self.__points[index]

    def get_closest_points(self, test_point: Point, correction_point_distance: int) -> "tuple[Point, Point]":
        """
        Return the closest and the correction point to a specific point.
        """
        distances:"list[float]" = []
        min_dist = -1
        index = 0

        for point in self.__points:
            distances.append(point.get_distance(test_point))
        
        for i in range(len(distances)):
            if min_dist == -1 or distances[i] < min_dist:
                min_dist = distances[i]
                index = i

        correction_point = index + correction_point_distance
        if(correction_point >= len(distances)):
            correction_point = len(distances)-1

        return self.__points[index], self.__points[correction_point]