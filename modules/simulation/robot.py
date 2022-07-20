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
        Move the robot forward.
        """
        if(self.quadrant == 1 or self.quadrant == 2):
            new_x = round(last_lon + self.delta_x, 5)
        else:
            new_x = round(last_lon - self.delta_x, 5)
        new_y = self.get_y(new_x)
        return new_y, new_x

    def turn_left(self, last_lon: float) -> "tuple[float, float]":
        """
        Turn the robot to the left by 5 degrees.
        """
        # Update the line angular coefficient
        if(self.quadrant == 1 or self.quadrant == 3):
            self.update_slope(self.slope_degrees - 5)
        elif(self.quadrant == 2 or self.quadrant == 4):
            self.update_slope(self.slope_degrees + 5)            
        else: # infinite
            self.update_slope(self.slope_degrees + 5)

        return self.forward(last_lon)

    def turn_right(self, last_lon: float) -> "tuple[float, float]":
        """
        Turn the robot to the right by 5 degrees.
        """
        # Update the line angular coefficient
        if(self.quadrant == 1 or self.quadrant == 3):
            self.update_slope(self.slope_degrees + 5)
        elif(self.quadrant == 2 or self.quadrant == 4):
            self.update_slope(self.slope_degrees - 5)            
        else: # infinite
            self.update_slope(self.slope_degrees + 5)

        return self.forward(last_lon)

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

r = Robot(-25.43548, -54.59671, -25.43545, -54.59669)

print(-25.43548, -54.59671)
print(-25.43545, -54.59669)

new_lat, new_lon = r.forward(-54.59669)
print(new_lat, new_lon, r.slope, r.slope_degrees)
new_lat, new_lon = r.turn_left(new_lon)
print(new_lat, new_lon, r.slope, r.slope_degrees)
new_lat, new_lon = r.turn_right(new_lon)
print(new_lat, new_lon, r.slope, r.slope_degrees)
new_lat, new_lon = r.forward(new_lon)
print(new_lat, new_lon, r.slope, r.slope_degrees)
#new_lat, new_lon = r.turn_right(new_lon)
#print(new_lat, new_lon, r.slope, r.slope_degrees)
#new_lat, new_lon = r.turn_right(new_lon)
#print(new_lat, new_lon, r.slope, r.slope_degrees)










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