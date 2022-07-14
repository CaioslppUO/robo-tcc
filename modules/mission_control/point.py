from auto_mode_calcs import dist_two_points
class Point():
    def __init__(self, latitude: float, longitude: float, id: int = 0) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.id = 0

class Points():
    def __init__(self) -> None:
        self.__points: "list[Point]" = []

    def add_point(self, latitude: float, longitude: float) -> None:
        self.__points.append(Point(latitude, longitude, len(self.__points)))

    def get_latitudes(self) -> "tuple[float]":
        latitudes: "list[float]" = []
        for point in self.__points:
            latitudes.append(point.latitude)
        return latitudes

    def get_longitudes(self) -> "tuple[float]":
        longitudes: "list[float]" = []
        for point in self.__points:
            longitudes.append(point.longitude)
        return longitudes

    def get_closest_points(self, point: Point, correction_point_distance: int) -> Point:
        dist:"list[float]" = []
        minDist = -1
        minIndex = 0

        for i in self.__points:
            dist.append(dist_two_points(point.latitude, point.longitude, i.latitude, i.longitude))

        for i in range(len(dist)):
            if minDist == -1 or dist[i] < minDist:
                minDist = dist[i]
                minIndex = i

        correction_point = minIndex + correction_point_distance
        while (correction_point >= len(dist)):
            correction_point -= 1

        return self.__points[minIndex], self.__points[correction_point] 