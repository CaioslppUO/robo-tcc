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

    def get_point(self, index: int) -> Point:
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
            raise Exception("At least 2 points are needed to calculate the linear coefficient")

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

    def y_line_equation(self, x: float) -> float:
        """
        Return the Y value given the X. The line must have at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to calculate y given x")

        # Calculating Coefficients
        angular_coefficient = self.angular_coefficient()
        linear_coefficient = self.linear_coefficient()

        # Calculating Y
        y = angular_coefficient * x + linear_coefficient

        return round(y, self.__decimals)

    def x_line_equation(self, y: float) -> float:
        """
        Return the X value given the Y. The line must have at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to calculate x given y")

        # Calculating Coefficients
        angular_coefficient = self.angular_coefficient()
        linear_coefficient = self.linear_coefficient()

        # Calculating X
        x = (y - linear_coefficient) / angular_coefficient

        return round(x, self.__decimals)

    def is_above_the_line(self, point: Point) -> bool:
        """
        Return True if the point is above the line. The line must have at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to check if the point is above the line")

        # Calculating Coefficients
        angular_coefficient = self.angular_coefficient()
        linear_coefficient = self.linear_coefficient()
        
        # Checking
        return point.latitude > point.longitude * angular_coefficient + linear_coefficient

    def is_in_the_line(self, point: Point) -> bool:
        """
        Return True if the point is in the line. The line must have at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to check if the point is in the line")

        # Calculating Coefficients
        angular_coefficient = self.angular_coefficient()
        linear_coefficient = self.linear_coefficient()
        
        # Checking
        return point.latitude == point.longitude * angular_coefficient + linear_coefficient

    def is_under_the_line(self, point: Point) -> bool:
        """
        Return True if the point is under the line. The line must have at least 2 points.
        """
        if(len(self.points) < 2):
            raise Exception("At least 2 points are needed to check if the point is under the line")

        # Calculating Coefficients
        angular_coefficient = self.angular_coefficient()
        linear_coefficient = self.linear_coefficient()
        
        # Checking
        return point.latitude < point.longitude * angular_coefficient + linear_coefficient