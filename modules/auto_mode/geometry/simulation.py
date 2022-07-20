from re import A
from point import Point
from points import Points
from path_calculator import PathCalculator
from graph import plot
import random
from robot import Robot

class Simulation:
    def __init__(self) -> None:
        # Up Right
        self.start = Point(-25.435348, -54.596970)
        self.end = Point(-25.435324, -54.596960)

        # Down right
        #self.start = Point(-25.435348, -54.596970)
        #self.end = Point(-25.435355, -54.596960)
        self.path = PathCalculator(self.start, self.end, 34)
        self.points = self.path.get_points_between()

        self.move_factor = 0.000001
        self.move_factor_randomizer = 0.0000005

        self.robot = Robot(-25.435348, -54.596970, -25.4353477, -54.596969)

        self.robot_points = Points()
        # Up Right
        #self.robot_points.add_point(-25.435348, -54.596970)
        print(-25.4353477, -54.596969)
        self.robot_points.add_point(-25.4353477, -54.596969)

    def get_new_point(self, direction: str, last_point: Point, mission_quadrant: int) -> Point:
        """
        Calculate the new robot point.
        """
        much_smaller_slope = self.robot.slope < self.path.get_angular_coefficient()/2
        much_bigger_slope = self.robot.slope > self.path.get_angular_coefficient()*2
        perfect_slope = self.robot.slope >= self.path.get_angular_coefficient() - 0.3 and self.robot.slope <= self.path.get_angular_coefficient() + 0.3
        if(much_smaller_slope or much_bigger_slope):
            turn_angle = 10
        elif(perfect_slope):
            turn_angle = 3
        else:
            turn_angle = None
        if(direction == "forward"):
            lat, lon = self.robot.forward(last_point.longitude, mission_quadrant)
            print(lat, lon)
            return Point(lat, lon)
        elif(direction == "right"):
            lat, lon = self.robot.turn_right(last_point.longitude, mission_quadrant, turn_angle)
            print(lat, lon)
            return Point(lat, lon)
        elif(direction == "left"):
            lat, lon = self.robot.turn_left(last_point.longitude, mission_quadrant, turn_angle)
            print(lat, lon)
            return Point(lat, lon)

    def run(self, random_start_direction: bool = False, random_movement_factor: bool = False) -> None:
        """
        Execute the simulation.
        """
        reached_the_end = False
        for r_point in self.robot_points.get_points():
            print("Ponto: {}".format(r_point.id))
            print("cof_ang_robo: ", self.robot.slope)

            _a, _b = self.points.get_closest_points(r_point, 3)
            closest_point = Point(_a.latitude, _a.longitude)
            correction_point = Point(_b.latitude, _b.longitude)
            if(reached_the_end):
                plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())
                break
            correction_direction = r_point.get_correction_direction(self.start, self.path.get_angular_coefficient(), self.path.get_mission_quadrant(), closest_point)
            new_point = self.get_new_point(correction_direction, r_point, self.path.get_mission_quadrant())
            if(new_point != None):
                self.robot_points.add_point(new_point.latitude, new_point.longitude)
                reached_the_end, dist_to_end = self.end.equals(new_point, 0.00000150)
            else:
                print("Nao tem mais pontos")

            plot(self.points, robot=r_point, closest_point=closest_point, correction_point=correction_point, correction_direction=correction_direction, mission_quadrant=self.path.get_mission_quadrant())

    def run_2(self, random_start_direction: bool = False, random_movement_factor: bool = False) -> None:
        """
        Execute the simulation.
        """
        reached_the_end = False
        for r_point in self.robot_points.get_points():
            #print("Ponto: {}".format(r_point.id))
            #print("Graus: ", self.robot.slope_degrees)
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
                print("Nao tem mais pontos")
        print("Pontos: ", len(self.robot_points.get_points()))
        plot(self.points, robot_points=self.robot_points, mission_quadrant=self.path.get_mission_quadrant())


s = Simulation()

s.run_2()