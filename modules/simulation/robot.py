"""
@package robot.py
Simulation of the robot itself and its attributes.
"""

import math
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')

class Robot:
    def __init__(self, latitude_0: float = 0.0, longitude_0: float = 0.0, latitude_1: float = 0.0, longitude_1: float = 0.0) -> None:
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

        self.slope_degrees_with_signal = round(math.degrees(math.atan2(self.delta_y, self.delta_x)), 1)
        self.__get_quadrant()

        if(self.slope_degrees_with_signal < 0):
            self.slope_degrees =  self.slope_degrees_with_signal + 180
        else:
            self.slope_degrees = self.slope_degrees_with_signal

    def __get_quadrant(self) -> None:
        """
        Update the quadrant of the movement.
        """
        if(self.slope_degrees_with_signal > 0 and self.slope_degrees_with_signal < 90):
            self.quadrant = 1
        elif(self.slope_degrees_with_signal < 0 and self.slope_degrees_with_signal > -90):
            self.quadrant = 2
        elif(self.slope_degrees_with_signal < -90 and self.slope_degrees_with_signal > -180):
            self.quadrant = 3
        elif(self.slope_degrees_with_signal == 0 or self.slope_degrees_with_signal == 180 or self.slope_degrees_with_signal == -180):
            self.quadrant = None
        else:
            self.quadrant = 4

    def get_y(self, x: float) -> float:
        """
        Line equation.
        """
        return round(self.slope * x + self.linear_coefficient, 5)

    def forward(self, last_lon: float) -> "tuple[float, float]":
        """
        Move the robot forward using the speed_factor.
        """
        if(self.quadrant == 1 or self.quadrant == 2):
            new_x = last_lon + self.delta_x
        else:
            new_x = last_lon - self.delta_x
        new_y = self.get_y(new_x)
        return new_x, new_y

    def update_slope(self, new_slope_in_degrees: float) -> None:
        """
        Update the line to the new slope.
        """
        self.slope = round(math.tan(math.radians(new_slope_in_degrees)), 5)
        self.linear_coefficient = self.robot_point_0_latitude - self.robot_point_0_longitude * self.slope

        self.slope_degrees_with_signal = round(math.degrees(math.atan(self.slope)), 1)
        self.__get_quadrant()

        if(self.slope_degrees_with_signal < 0):
            self.slope_degrees =  self.slope_degrees_with_signal + 180
        else:
            self.slope_degrees = self.slope_degrees_with_signal
            
        print("Slope:", self.slope)
        print("Slope degrees:", self.slope_degrees)
        print("Slope degrees with signal:", self.slope_degrees_with_signal)
        print("Slope degrees converted to radians", round(math.radians(self.slope_degrees), 5))
        print("Slope degrees converted to radians tan", round(math.tan(math.radians(self.slope_degrees)), 5))

#r = Robot(-25.43548, -54.59671, -25.43545, -54.59669)

#r.forward(-54.59669)

r = Robot(0, 0, -1, 1)
#r.update_slope(1.0)
print("Point: ", r.forward(0))
print(r.slope)
print(r.slope_degrees)
print(r.slope_degrees_with_signal)
print(r.quadrant)
r.update_slope(r.slope_degrees + 5)











#xs, ys = [], []

#xs.append(-54.59671)
#ys.append(-25.43548)
#print(-54.59671, -25.43548)
#
#new_x, new_y = r.forward()
#xs.append(new_x)
#ys.append(new_y)
#print(new_x, new_y)
#
#r.latitude = new_y
#r.longitude = new_x
#
#new_x, new_y = r.forward()
#xs.append(new_x)
#ys.append(new_y)
#print(new_x, new_y)
#
#fig = plt.figure()
#ax = plt.axes()
#plt.xlabel("LON")
#plt.ylabel("LAT")
#
#plt.scatter(x=xs, y=ys)
#
#plt.axvline(x=0, c="red", label="x=0")
#plt.axhline(y=0, c="yellow", label="y=0")
#
#plt.show()