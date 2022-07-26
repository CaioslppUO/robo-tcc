from .point import Point
import math

class Line:
    def __init__(self, p1: Point, p2: Point, decimals: int = 7) -> None:
        self.__decimals = decimals

        # Points
        self.p1 = Point(p1.latitude, p1.longitude)
        self.p2 = Point(p2.latitude, p2.longitude)

        if(p1.equal(p2.latitude, p2.longitude)):
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
        pdf_lat, pdf_lon = self.p1.difference(self.p2.latitude, self.p2.longitude)

        # Calculating the Quadrant
        if(pdf_lat >= 0 and pdf_lon >= 0):
            return 1
        elif(pdf_lat >= 0 and pdf_lon <= 0):
            return 4
        elif(pdf_lat <= 0 and pdf_lon >= 0):
            return 2
        elif(pdf_lat <= 0 and pdf_lon <= 0):
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
        if(degrees == 90 or degrees == -90):
            return float("inf")
        return round(math.tan(math.radians(degrees)), self.__decimals)

    def __quadrant_has_changed(self, old_slope: float, new_slope: float) -> bool:
        """
        Return true if the quadrant has changed. Only detect changes under 90 degrees.
        """
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
                if(quadrant == 4):
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
            degree_slope -= 0.5
        new_slope = self.degrees_to_slope(degree_slope)

        # Updating the new line equation
        old_slope = self.angular_coefficient
        self.angular_coefficient = new_slope
        self.linear_coefficient = self.__linear_coefficient()

        self.__get_new_p2(self.quadrant, old_slope, new_slope, True)
        self.quadrant = self.__quadrant()

    def counter_clockwise_slope(self, dec_in_degrees: float) -> None:
        """
        Decrease the slope with dec_in_degrees (with signal).
        """
        # Calculating the new slope
        degree_slope = self.slope_to_degrees()
        degree_slope += dec_in_degrees
        if(degree_slope == 90 or degree_slope == 0 or degree_slope == -90):
            degree_slope += 0.5
        new_slope = self.degrees_to_slope(degree_slope)

        # Updating the new line equation
        old_slope = self.angular_coefficient
        self.angular_coefficient = new_slope
        self.linear_coefficient = self.__linear_coefficient()

        self.__get_new_p2(self.quadrant, old_slope, new_slope, False)
        self.quadrant = self.__quadrant()

    def get_smaller_rotation_direction(self, objective_line_p1: Point, obejctive_line_p2: Point) -> str:
        """
        Return the smaller rotation direction (clockwise, counter_clockwise) to reach objective_line from self.
        """
        objective_line = Line(objective_line_p1, obejctive_line_p2)
        common_x = objective_line.p2.longitude

        y_objective = objective_line.y_line_equation(common_x)
        y_actual = self.y_line_equation(common_x)

        actual_quadrant = self.quadrant
        objective_quadrant = objective_line.quadrant

        if(y_actual == y_objective):
            if(actual_quadrant == objective_quadrant):
                return "none"
            else:
                return "clockwise"

        if(objective_line.quadrant == 1 or objective_line.quadrant == 2):
            if(y_actual > y_objective and self.angular_coefficient >= objective_line.angular_coefficient): # Actual above objective
                if(actual_quadrant == 1):
                    return "clockwise"
                elif(actual_quadrant == 3):
                    return "counter_clockwise"
                elif(actual_quadrant == 2):
                    return "counter_clockwise"
                elif(actual_quadrant == 4):
                    return "clockwise"
                return "nao deveria ter caido aqui"
                raise Exception("Could not determine rotation direction for actual above objective")
            else: # Actual under objective
                if(actual_quadrant == 2):
                    return "counter_clockwise"
                elif(actual_quadrant == 4):
                    return "clockwise"
                elif(actual_quadrant == 1):
                    return "clockwise"
                elif(actual_quadrant == 3):
                    return "counter_clockwise"
                else:
                    return "nao deveria ter caido aqui"
                    #raise Exception("Could not determine rotation direction for actual under objective")
        else:
            if(y_actual > y_objective): # Actual above objective
                if(actual_quadrant == 2):
                    return "clockwise"
                elif(actual_quadrant == 4):
                    return "counter_clockwise"
                elif(actual_quadrant == 1):
                    return "counter_clockwise"
                elif(actual_quadrant == 3):
                    return "clockwise"
                return "nao deveria ter caido aqui"
                raise Exception("Could not determine rotation direction for actual above objective")
            else: # Actual under objective
                if(actual_quadrant == 1):
                    return "counter_clockwise"
                elif(actual_quadrant == 3):
                    return "clockwise"
                elif(actual_quadrant == 4):
                    return "counter_clockwise"
                elif(actual_quadrant == 2):
                    return "clockwise"
                else:
                    return "nao deveria ter caido aqui"
                    raise Exception("Could not determine rotation direction for actual under objective")