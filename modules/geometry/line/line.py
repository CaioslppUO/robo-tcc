from .point import Point

class Line:
    def __init__(self, decimals: int = 7) -> None:
        self.points: "list[Point]" = []
        self.__decimals = decimals

    def add(self, point: Point) -> None:
        """
        Add a new point to the line.
        """
        point.id = len(self.points)
        self.points.append(point)

    def get(self, index: int) -> Point:
        """
        Return the point a index.
        """
        return self.points[index]

    def angular_coefficient(self) -> float:
        """
        Calculate the angular coefficient if the line has at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to calculate the angular coefficient")

        # Selecting Points
        p1 = self.points[0]
        p2 = self.points[1]
        
        # Calculating Deltas
        delta_y = p2.latitude - p1.latitude
        delta_x = p2.longitude - p1.longitude

        # Angular Coefficient
        return round(delta_y / delta_x, self.__decimals)

    def linear_coefficient(self) -> float:
        """
        Return the linear coefficient if the line has at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to calculate the angular coefficient")

        # Calculating Angular Coefficient
        angular_coefficient = self.angular_coefficient()

        # Selecting Point
        p = self.points[0]

        # Linear Coefficient
        return round(p.latitude - p.longitude * angular_coefficient, self.__decimals)

    def closest_point(self, test_point: Point) -> Point:
        """
        Return the closest point to test_point.
        """
        distances: "list[float]" = []
        min_dist = -1
        index = 0

        for point in self.points:
            distances.append(test_point.distance(point))

        for i in range(len(distances)):
            if(min_dist == -1 or distances[i] < min_dist):
                min_dist = distances[i]
                index = i
        
        return self.points[index]