from re import A
from point import Point
from points import Points
from path_calculator import PathCalculator
from graph import plot
import random
from robot import Robot

class Simulation:
    def __init__(self) -> None:
        self.__break_limit = 50                

        self.move_factor = 0.000001
        self.move_factor_randomizer = 0.0000005

        self.robot_points = Points()

        ## Up right
        #r_a, r_e , r_2 = Point(-25.435348, -54.596970), Point(-25.435324, -54.596960), Point(-25.4353477, -54.596969)
        #self.start = r_a
        #self.end = r_e
        #self.robot = Robot(r_a.latitude, r_a.longitude, r_2.latitude, r_2.longitude)
        #self.robot_points.add_point(r_2.latitude, r_2.longitude)

        # Down right
        #r_a, r_e, r_2 = Point(-25.435348, -54.596970), Point(-25.435355, -54.596960), Point(-25.435348, -54.596969)
        #self.start = r_a
        #self.end = r_e
        #self.robot = Robot(r_a.latitude, r_a.longitude, r_2.latitude, r_2.longitude)
        #self.robot_points.add_point(r_2.latitude, r_2.longitude)

        # Up left
        #r_a, r_e, r_2 = Point(-25.435348, -54.596970), Point(-25.435324, -54.5969705), Point(-25.435347, -54.5969701)
        #self.start = r_a
        #self.end = r_e
        #self.robot = Robot(r_a.latitude, r_a.longitude, r_2.latitude, r_2.longitude)
        #self.robot_points.add_point(r_2.latitude, r_2.longitude)

        # Down left
        r_a, r_e, r_2 = Point(-25.435348, -54.596970), Point(-25.435355, -54.5969705), Point(-25.435348, -54.5969701)
        self.start = r_a
        self.end = r_e
        self.robot = Robot(r_a.latitude, r_a.longitude, r_2.latitude, r_2.longitude)
        self.robot_points.add_point(r_2.latitude, r_2.longitude)

        # Path calculator
        self.path = PathCalculator(self.start, self.end, 34)
        self.points = self.path.get_points_between()

        print("Start Point:                   ({:10.8f}, {:10.8f})".format(self.start.latitude, self.start.longitude))
        print("End Point:                     ({:10.8f}, {:10.8f})".format(self.end.latitude, self.end.longitude))
        print("Mission Quadrant:              {}".format(self.path.get_mission_quadrant()))
        print("Points Between:                {}".format(len(self.points.get_points())))
        print("*"*55)       
        print("Initial Robot Slope:           {}".format(self.robot.slope))
        print("Initial Robot Slope (Degrees): {}°".format(self.robot.slope_degrees))
        print("initial Direction Quadrant:    {}".format(self.robot.get_quadrant()))

    def get_new_point(self, direction: str, last_point: Point, mission_quadrant: int, closest_mission_point: Point) -> Point:
        """
        Calculate the new robot point.
        """
        much_smaller_slope = self.robot.slope < self.path.get_angular_coefficient()*2
        much_bigger_slope = self.robot.slope > self.path.get_angular_coefficient()*2
        perfect_slope = self.robot.slope >= self.path.get_angular_coefficient() - 0.3 and self.robot.slope <= self.path.get_angular_coefficient() + 0.3
        is_distant = last_point.get_distance(closest_mission_point) >= 0.0000001

        lat, lon = 0, 0

        print(is_distant)
        if((much_bigger_slope or much_smaller_slope) and is_distant):
            turn_angle = 11
        elif(perfect_slope):
            turn_angle = 3
        else:
            turn_angle = None
        if(direction == "forward"):
            lat, lon = self.robot.forward(last_point.longitude)
        elif(direction == "right"):
            lat, lon = self.robot.turn_right(last_point, turn_angle)
        elif(direction == "left"):
            lat, lon = self.robot.turn_left(last_point, turn_angle)

        if(abs(lat) < 25 or abs(lat) > 26 or abs(lon) < 54 or abs(lon) > 55):
            return last_point
        return Point(lat, lon)

    def run(self, random_start_direction: bool = False, random_movement_factor: bool = False) -> None:
        """
        Execute the simulation.
        """
        reached_the_end = False
        for r_point in self.robot_points.get_points():
            if(reached_the_end):
                plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())
                break
            
            # Getting closest point to r_point
            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
            
            # Getting the next point
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant(), closest_point)
            
            print("-"*55)
            print("Robot Point:              ({:10.8f}, {:10.8f})".format(r_point.latitude, r_point.longitude))
            print("Robot Direction Quadrant: {}".format(self.robot.get_quadrant()))
            print("Robot Slope:              {}".format(self.robot.slope))
            print("Robot Slope (Degrees):    {}°".format(self.robot.slope_degrees))
            print("Robot Next Direction:     {}".format(correction_direction))
            
            new_point = self.get_new_point(correction_direction, r_point, self.path.get_mission_quadrant(), closest_point)
            
            # Adding the next point to points list
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
                reached_the_end, dist_to_end = self.end.equals(new_point, 0.00000150)

            plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())

    def run_2(self, random_start_direction: bool = False, random_movement_factor: bool = False) -> None:
        """
        Execute the simulation.
        """
        reached_the_end = False
        i = 0
        for r_point in self.robot_points.get_points():
            if(i == self.__break_limit):
                break
            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
            if(reached_the_end):
                break
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant(), closest_point)
            new_point = self.get_new_point(correction_direction, r_point, self.path.get_mission_quadrant())
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
                reached_the_end, dist_to_end = self.end.equals(new_point, 0.00000150)
            else:
                #print("Nao tem mais pontos")
                pass
            i += 1
        print("Pontos: ", len(self.robot_points.get_points()))
        plot(self.points, robot_points=self.robot_points, mission_quadrant=self.path.get_mission_quadrant())


s = Simulation()

s.run()