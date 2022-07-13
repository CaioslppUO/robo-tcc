from auto_mode_calcs import dist_two_points

class Point:
    def __init__(self,latitude:float,longitude:float,id:int) -> None:
        self.latitude:float = latitude
        self.longitude:float = longitude
        self.id:int = id

class Points:
    def __init__(self) -> None:
        self.points:"list[Point]" = []

    def add_point(self,latitude:float,longitude:float) -> None:
        self.points.append(Point(latitude,longitude,len(self.points)))

    def add_points(self, latitudes: "list[float]", longitudes: "list[float]") -> None:
        i = 0
        for _ in latitudes:
            self.points.append(Point(latitudes[i], longitudes[i], len(self.points)))
            i += 1
    
    def get_order_by_distance(self, point: Point) -> Point:
        dist:"list[float]" = []
        minDist = -1
        minIndex= 0

        for i in self.points:
            dist.append(dist_two_points(point.latitude, point.longitude, i.latitude, i.longitude))

        for i in range(len(dist)):
            if minDist == -1 or dist[i] < minDist:
                minDist = dist[i]
                minIndex = i

        correction_point = minIndex + 5
        while (correction_point >= len(dist)):
            correction_point -= 1

        return self.points[minIndex], self.points[correction_point]