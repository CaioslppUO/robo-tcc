from re import A
from point import Point
from points import Points
from path_calculator import PathCalculator
from graph import plot

class Simulation:
    def __init__(self) -> None:
        self.start = Point(-25.435348, -54.596970)
        self.end = Point(-25.435324, -54.596960)
        self.path = PathCalculator(self.start, self.end, 15)
        self.points = self.path.get_points_between()

        self.move_factor = 0.000001

        self.axis = "y"
        self.direction = "up"

        self.robot_points = Points()
        self.robot_points.add_point(-25.435348, -54.596970)

    def get_new_point(self, direction: str, last_point: Point) -> Point:
        """
        Calculate the new robot point.
        """
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

    def run(self) -> None:
        """
        Execute the simulation.
        """
        last_correction_point = "forward"
        for r_point in self.robot_points.get_points():
            print("Ponto: {}".format(r_point.id))
            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
        
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant())
            if(last_correction_point != correction_direction):
                last_correction_point = correction_direction
            else:
                correction_direction = "forward"
            new_point = self.get_new_point(correction_direction, self.robot_points.get_points()[-1])
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
            
            plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())

s = Simulation()

s.run()