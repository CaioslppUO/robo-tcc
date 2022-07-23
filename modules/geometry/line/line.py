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

        if(delta_x == 0):
            return float("inf")

        # Angular Coefficient
        return round(delta_y / delta_x, self.__decimals)

    def __linear_coefficient(self) -> float:
        """
        Return the linear coefficient.
        """
        if(self.angular_coefficient == float("inf")):
            return round(self.p1.latitude, self.__decimals)
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
        Return which quadrant the line is pointing to. 5,6 is (+X,-X) axis and 7,8 is (+Y,-Y) axis.
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
        elif(pdf.latitude == 0 and pdf.longitude > 0):
            return 5
        elif(pdf.latitude == 0 and pdf.longitude < 0):
            return 6
        elif(pdf.latitude > 0 and pdf.longitude == 0):
            return 7
        elif(pdf.latitude < 0 and pdf.longitude == 0):
            return 8
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
        if(degrees == 90 or degrees == -90):
            return float("inf")
        return round(math.tan(math.radians(degrees)), self.__decimals)

    def __quadrant_has_changed(self, old_slope: float, new_slope: float) -> bool:
        """
        Return true if the quadrant has changed. Only detect changes under 90 degrees.
        """
        if(old_slope == float("inf") and new_slope != float("inf")):
            return True
        if(old_slope != float("inf") and new_slope == float("inf")):
            return True
        if(old_slope == 0 and new_slope != 0):
            return True
        if(old_slope != 0 and new_slope == 0):
            return True
        if(old_slope > 0 and new_slope > 0):
            return False
        if(old_slope > 0 and new_slope < 0):
            return True
        if(old_slope < 0 and new_slope > 0):
            return True
        return False

    def __get_new_p2(self, quadrant: int, old_slope: float, new_slope: float, clockwise: bool) -> None:
        """
        Get a new P2 point.
        """
        # Updating the quadrant
        if(self.__quadrant_has_changed(old_slope, new_slope)):
            if(clockwise):
                if(new_slope == float("inf")): # Is in an axis
                    if(quadrant == 1):
                        new_quadrant = 5
                    elif(quadrant == 2):
                        new_quadrant = 8
                    elif(quadrant == 3):
                        new_quadrant = 6
                    elif(quadrant == 4):
                        new_quadrant = 7
                    else:
                        raise Exception("Could not define quadrant for infinite new slope")
                if(quadrant > 4):
                    if(quadrant == 5):
                        new_quadrant = 2
                    elif(quadrant == 6):
                        new_quadrant = 4
                    elif(quadrant == 7):
                        new_quadrant = 1
                    elif(quadrant == 8):
                        new_quadrant = 3
                    else:
                        raise Exception("Could not define quadrant bigger than 4")
                elif(quadrant == 4):
                    new_quadrant = 1
                else:
                    new_quadrant = quadrant + 1
            else:
                if(quadrant == 1):
                    new_quadrant = 4
                else:
                    new_quadrant = quadrant - 1
        else:
            new_quadrant = quadrant

        # Getting a New P2
        delta_x = abs(self.p2.longitude - self.p1.longitude)
        delta_y = abs(self.p2.latitude - self.p1.latitude)

        if(delta_x != 0):
            if(new_quadrant == 1 or new_quadrant == 2 or new_quadrant == 5):
                new_x = delta_x
            else:
                new_x = -delta_x
            new_y = self.y_line_equation(new_x)
        elif(delta_y != 0):
            if(new_quadrant == 1 or new_quadrant == 4 or new_quadrant == 7):
                new_y = delta_y
            else:
                new_y = -delta_y
            new_x = self.x_line_equation(new_y)

        self.p2.longitude = new_x
        self.p2.latitude = new_y

    def clockwise_slope(self, inc_in_degrees: float) -> None:
        """
        Increase the slope with inc_in_degrees (with signal).
        """
        # Calculating the new slope
        degree_slope = self.slope_to_degrees()
        degree_slope -= inc_in_degrees
        if(degree_slope == 90 or degree_slope == 0 or degree_slope == -90):
            degree_slope -= 0.01
        new_slope = self.degrees_to_slope(degree_slope)

        # Updating the new line equation
        old_slope = self.angular_coefficient
        self.angular_coefficient = new_slope
        self.linear_coefficient = self.__linear_coefficient()

        self.__get_new_p2(self.quadrant, old_slope, new_slope, True)
        self.quadrant = self.__quadrant()