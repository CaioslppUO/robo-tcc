from re import A
from point import Point
from points import Points
from path_calculator import PathCalculator
from graph import plot
import random

class Simulation:
    def __init__(self) -> None:
        # Up Right
        #self.start = Point(-25.435348, -54.596970)
        #self.end = Point(-25.435324, -54.596960)

        # Down right
        self.start = Point(-25.435348, -54.596970)
        self.end = Point(-25.435355, -54.596960)
        self.path = PathCalculator(self.start, self.end, 34)
        self.points = self.path.get_points_between()

        self.move_factor = 0.000001
        self.move_factor_randomizer = 0.0000005

        self.axis = "y"
        self.direction = "up"

        self.robot_points = Points()
        # Up Right
        #self.robot_points.add_point(-25.435348, -54.596970)
        self.robot_points.add_point(-25.435348, -54.596970)

    def set_random_direction_and_axis(self) -> None:
        """
        Set the direction and axis to a random value (up or down), (y or x).
        """
        i = random.randint(0, 1)
        if(i == 0):
            self.direction = "up"
            j = random.randint(0, 1)
            if(j == 0):
                self.axis = "y"
            else:
                self.axis = "x"
        else:
            self.direction = "down"
            j = random.randint(0, 1)
            if(j == 0):
                self.axis = "y"
            else:
                self.axis = "x"

    def get_new_point(self, direction: str, last_point: Point, random_move: bool) -> Point:
        """
        Calculate the new robot point.
        """
        move_factor = self.move_factor
        if(random):
            i = random.randint(0, 1)
            if(i == 0):
                move_factor += self.move_factor_randomizer
            else:
                move_factor -= self.move_factor_randomizer
                
        if(direction == "forward"):
            if(self.axis == "y"):
                if(self.direction == "up"):
                    return Point(last_point.latitude + self.move_factor, last_point.longitude)
                else:
                    return Point(last_point.latitude - self.move_factor, last_point.longitude)
            elif(self.axis == "x"):
                if(self.direction == "up"):
                    return Point(last_point.latitude, last_point.longitude + self.move_factor)
                else:
                    return Point(last_point.latitude, last_point.longitude - self.move_factor)

        elif(direction == "right"):
            if(self.axis == "y"):
                self.axis = "x"
                if(self.direction == "up"):
                    return Point(last_point.latitude, last_point.longitude + self.move_factor)
                else:
                    return Point(last_point.latitude, last_point.longitude - self.move_factor)                    
            elif(self.axis == "x"):
                self.axis = "y"
                if(self.direction == "up"):
                    self.direction = "down"
                    return Point(last_point.latitude - self.move_factor, last_point.longitude)
                else:
                    self.direction = "up"
                    return Point(last_point.latitude + self.move_factor, last_point.longitude)

        else: # direction = left
            if(self.axis == "y"):
                self.axis = "x"
                if(self.direction == "up"):
                    self.direction = "down"
                    return Point(last_point.latitude, last_point.longitude - self.move_factor)
                else:
                    self.direction = "up"
                    return Point(last_point.latitude, last_point.longitude + self.move_factor)
            elif(self.axis == "x"):
                self.axis = "y"
                if(self.direction == "up"):
                    return Point(last_point.latitude + self.move_factor, last_point.longitude)
                else:
                    return Point(last_point.latitude - self.move_factor, last_point.longitude)

    def run(self, random_start_direction: bool = False, random_movement_factor: bool = False) -> None:
        """
        Execute the simulation.
        """
        last_correction_point = "forward"
        if(random_start_direction):
            self.set_random_direction_and_axis()
        print(self.direction, self.axis)
        for r_point in self.robot_points.get_points():
            print("Ponto: {}".format(r_point.id))
            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
        
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant(), closest_point)
            #if(last_correction_point != correction_direction):
            #    last_correction_point = correction_direction
            #else:
            #    correction_direction = "forward"
            new_point = self.get_new_point(correction_direction, self.robot_points.get_points()[-1], random_move=random_movement_factor)
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
                if(self.end.equals(new_point, 0.0000001)):
                    break
            #plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())

    def run_2(self) -> None:
        """
        Execute the simulation.
        """
        last_correction_point = "forward"
        n_points = 0
        for r_point in self.robot_points.get_points():
            print("Ponto: {}".format(r_point.id))
            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
        
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant(), closest_point)
            #if(last_correction_point != correction_direction):
            #    last_correction_point = correction_direction
            #else:
            #    correction_direction = "forward"
            new_point = self.get_new_point(correction_direction, self.robot_points.get_points()[-1], False)
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
                if(self.end.equals(new_point, 0.0000001)):
                    break
            if(n_points >= 200):
                break
            n_points += 1
            
        plot(self.points, robot_points=self.robot_points, mission_quadrant=self.path.get_mission_quadrant())

s = Simulation()

s.direction = "down"
s.axis = "y"

#s.run(random_start_direction=False, random_movement_factor=True)
s.run_2()