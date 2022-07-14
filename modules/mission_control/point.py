class Point():
    def __init__(self, latitude: float, longitude: float) -> None:
        self.latitude = latitude
        self.longitude = longitude

class Points():
    def __init__(self) -> None:
        self.__points: "list[Point]" = []

    def add_point(self, latitude: float, longitude: float) -> None:
        self.__points.append(Point(latitude, longitude))

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