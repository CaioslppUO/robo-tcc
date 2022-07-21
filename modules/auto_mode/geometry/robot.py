"""
@package robot.py
Simulation of the robot itself and its attributes.
"""

import math
import matplotlib.pyplot as plt

from point import Point
plt.style.use('seaborn-whitegrid')

class Robot:
    def __init__(self, latitude_0: float = 0.0, longitude_0: float = 0.0, latitude_1: float = 0.0, longitude_1: float = 0.0) -> None:
        # Factors
        self.decimals = 8
        self.turn_angle = 3
        self.turn_bigger_angle = 10

        # Localization
        ## First robot point
        self.robot_point_0_latitude: float = latitude_0
        self.robot_point_0_longitude: float = longitude_0
        ## Second robot point
        self.robot_point_1_latitude: float = latitude_1
        self.robot_point_1_longitude: float = longitude_1

        # Line equation values
        self.delta_x = self.robot_point_1_longitude - self.robot_point_0_longitude
        self.delta_y = self.robot_point_1_latitude - self.robot_point_0_latitude

        self.slope = self.delta_y / self.delta_x
        self.linear_coefficient = self.robot_point_0_latitude - self.robot_point_0_longitude * self.slope
        self.distance_in_longitude_between_points = self.delta_x/2

        self.slope_degrees_with_signal = round(math.degrees(math.atan2(self.delta_y, self.delta_x)), 1)
        self.__quadrant = self.__calculate_quadrant(Point(latitude_0, longitude_0), Point(latitude_1, longitude_1))

        if(self.slope_degrees_with_signal < 0):
            self.slope_degrees =  self.slope_degrees_with_signal + 180
        else:
            self.slope_degrees = self.slope_degrees_with_signal

    def __calculate_quadrant(self, p1: Point, p2: Point) -> None:
        """
        Update the quadrant of the movement.
        """
        if(p2.latitude > p1.latitude):
            if(p2.longitude > p1.longitude):
                return 1
            else:
                return 4
        else:
            if(p2.longitude > p1.longitude):
                return 2
            else:
                return 3

    def get_quadrant(self) -> int:
        return self.__quadrant

    def get_y(self, x: float) -> float:
        """
        Line equation.
        """
        return round(self.slope * x + self.linear_coefficient, self.decimals)

    def forward(self, last_lon: float) -> "tuple[float, float]":
        """
        Move the robot forward.
        """
        if(self.__quadrant == 1 or self.__quadrant == 2):
            print("Longitude Operation: +")
            new_x = round(last_lon + self.distance_in_longitude_between_points, self.decimals)
        else:
            print("Longitude Operation: -")
            new_x = round(last_lon - self.distance_in_longitude_between_points, self.decimals)
        new_y = self.get_y(new_x)
        return new_y, new_x

    def turn_left(self, last_point: Point, turn_angle: int = None) -> "tuple[float, float]":
        """
        Turn the robot to the left by 5 degrees.
        """
        if(turn_angle == None):
            turn_angle = self.turn_angle

        # Update the line angular coefficient
        if(self.__quadrant == 1 or self.__quadrant == 3):
            print("Angle Operation: +")
            self.update_slope(self.slope_degrees + turn_angle, last_point=last_point)
        elif(self.__quadrant == 2 or self.__quadrant == 4):
            print("Angle Operation: _")
            self.update_slope(self.slope_degrees + turn_angle, last_point=last_point)            
        else: # infinite
            print("Angle Operation: - (inf)")
            self.update_slope(self.slope_degrees - turn_angle, last_point=last_point)

        return self.forward(last_point.longitude)

    def turn_right(self, last_point: Point, turn_angle: int = None) -> "tuple[float, float]":
        """
        Turn the robot to the right by 5 degrees.
        """
        if(turn_angle == None):
            turn_angle = self.turn_angle

        # Update the line angular coefficient
        if(self.__quadrant == 1 or self.__quadrant == 3):
            print("Angle Operation: -")
            self.update_slope(self.slope_degrees - turn_angle, last_point=last_point)
        elif(self.__quadrant == 2 or self.__quadrant == 4):
            print("Angle Operation: -")
            self.update_slope(self.slope_degrees - turn_angle, last_point=last_point)            
        else: # infinite
            print("Angle Operation: - (inf)")
            self.update_slope(self.slope_degrees - turn_angle, last_point=last_point)

        return self.forward(last_point.longitude)

    def update_slope(self, new_slope_in_degrees: float, last_point: Point) -> None:
        """
        Update the line to the new slope.
        """
        self.slope = round(math.tan(math.radians(new_slope_in_degrees)), self.decimals)
        self.linear_coefficient = self.robot_point_0_latitude - self.robot_point_0_longitude * self.slope
        self.slope_degrees_with_signal = round(math.degrees(math.atan(self.slope)), 1)

        if(self.slope_degrees_with_signal < 0):
            self.slope_degrees =  self.slope_degrees_with_signal + 180
        else:
            self.slope_degrees = self.slope_degrees_with_signal

        if(self.__quadrant == 1):
            if(self.slope_degrees_with_signal <= 0 and self.slope_degrees_with_signal >= -self.turn_bigger_angle):
                self.__quadrant = 2
            elif(self.slope_degrees_with_signal <= 0 and self.slope_degrees_with_signal >= -90 + self.turn_bigger_angle):
                self.__quadrant = 4
        elif(self.__quadrant == 2):
            if(self.slope_degrees_with_signal >= 0 and self.slope_degrees_with_signal <= self.turn_bigger_angle):
                self.__quadrant = 1
            elif(self.slope_degrees_with_signal >= 0 and self.slope_degrees_with_signal <= 90 - self.turn_bigger_angle):
                self.__quadrant = 3
        elif(self.__quadrant == 3):
            if(self.slope_degrees_with_signal <= 0 and self.slope_degrees_with_signal >= -self.turn_bigger_angle):
                self.__quadrant = 4
            elif(self.slope_degrees_with_signal <= 0 and self.slope_degrees_with_signal >= -90 + self.turn_bigger_angle):
                self.__quadrant = 2
        else: # quadrant 4
            if(self.slope_degrees_with_signal >= 0 and self.slope_degrees_with_signal <= 90 - self.turn_bigger_angle):
                self.__quadrant = 1
            elif(self.slope_degrees_with_signal >= 0 and self.slope_degrees_with_signal <= -self.turn_bigger_angle):
                self.__quadrant = 3
