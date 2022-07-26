from .line import Line, Point

class PathCalculator:
    def __init__(self) -> None:
        pass


    def get_points_between(self,line_target: Line, number_of_points: int) -> "list[Point]":
        """
        Return number_of_points between start and end.
        """
        points: "list[Point]" = []
        longitude = line_target.p1.longitude
        for i in range(0, number_of_points - 1):
            latitude = round(line_target.angular_coefficient * longitude + line_target.linear_coefficient, 7)
            points.append(Point(latitude, longitude))
            if(line_target.p2.longitude > line_target.p1.longitude):
                longitude += line_target.p1.difference(line_target.p2.latitude, line_target.p2.longitude)[1] / (number_of_points - 1)
            else:
                longitude -= line_target.p1.difference(line_target.p2.latitude, line_target.p2.longitude)[1] / (number_of_points - 1)
        points.append(line_target.p2)
        return points


    def get_closest_point(self,line_target:"list[Point]", robot:Point) -> int:
        """
        Return the index of the closest point to the robot.
        """
        distances:"list[float]" = []
        min_dist = -1
        index = 0

        for point in line_target:
            distances.append(point.distance(robot.latitude, robot.longitude))
        for i in range(len(distances)):
            if min_dist == -1 or distances[i] < min_dist:
                min_dist = distances[i]
                index = i
        correction_point = index + 5
        if(correction_point >= len(distances)):
                correction_point = len(distances)-1

        return correction_point