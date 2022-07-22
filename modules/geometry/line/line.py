from .point import Point
import math

class Line:
    def __init__(self, p1: Point, p2: Point, decimals: int = 7) -> None:
        self.__decimals = decimals

        # Points
        self.p1 = Point(p1.latitude, p1.longitude)
        self.p2 = Point(p2.latitude, p2.longitude)

        if(p1.equal(p2)):
            raise Exception("Points must be different to form a line")

        # Calculations
        self.angular_coefficient = self.__angular_coefficient()
        self.linear_coefficient = self.__linear_coefficient()
        self.quadrant = self.__quadrant()

    def __angular_coefficient(self) -> float:
        """
        Calculate the angular coefficient.
        """
        # Calculating Deltas
        delta_y = self.p2.latitude - self.p1.latitude
        delta_x = self.p2.longitude - self.p1.longitude

        # Angular Coefficient
        return round(delta_y / delta_x, self.__decimals)

    def __linear_coefficient(self) -> float:
        """
        Return the linear coefficient.
        """
        return round(self.p1.latitude - self.p1.longitude * self.angular_coefficient, self.__decimals)

    def y_line_equation(self, x: float) -> float:
        """
        Return the Y value given the X.
        """
        y = self.angular_coefficient * x + self.linear_coefficient
        return round(y, self.__decimals)

    def x_line_equation(self, y: float) -> float:
        """
        Return the X value given the Y.
        """
        x = (y - self.linear_coefficient) / self.angular_coefficient
        return round(x, self.__decimals)

    def is_above(self, point: Point) -> bool:
        """
        Return True if the point is above the line.
        """
        return point.latitude > point.longitude * self.angular_coefficient + self.linear_coefficient

    def is_in(self, point: Point) -> bool:
        """
        Return True if the point is in the line.
        """
        return point.latitude == point.longitude * self.angular_coefficient + self.linear_coefficient

    def is_under(self, point: Point) -> bool:
        """
        Return True if the point is under the line.
        """
        return point.latitude < point.longitude * self.angular_coefficient + self.linear_coefficient

    def __quadrant(self) -> int:
        """
        Return which quadrant the line is pointing to.
        """
        # Calculating the Difference
        pdf = self.p1.difference(self.p2)

        # Calculating the Quadrant
        if(pdf.latitude > 0 and pdf.longitude > 0):
            return 1
        elif(pdf.latitude > 0 and pdf.longitude < 0):
            return 4
        elif(pdf.latitude < 0 and pdf.longitude > 0):
            return 2
        elif(pdf.latitude < 0 and pdf.longitude < 0):
            return 3
        raise Exception("Unknown quadrant in quadrant calculation")

    def slope_to_degrees(self, with_signal: bool = True) -> float:
        """
        Return the slope as a degree value.
        """
        degrees_with_signal = round(math.degrees(math.atan(self.angular_coefficient)), self.__decimals)
        
        if(not with_signal and degrees_with_signal < 0):
            return degrees_with_signal + 180
        
        return degrees_with_signal

    def degrees_to_slope(self, degrees: float) -> float:
        """
        Convert degrees (with signal) to slope.
        """
        if(degrees == 90):
            return float("inf")
        return round(math.tan(math.radians(degrees)), self.__decimals)

    def increase_slope(self, inc_in_degrees: float) -> None:
        """
        Increase the slope with inc_in_degrees (with signal).
        """
        degree_slope = self.slope_as_degrees()
        degree_slope += inc_in_degrees
