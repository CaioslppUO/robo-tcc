import math
from mission import _Location

def rad_to_deg(rad: float) -> float:
    """
    Convert radians to degrees.
    """
    return rad * 180 / math.pi

def dist_two_points(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the distance between two points.
    """
    f1 = (lat2 - lat1)**2
    f2 = (lon2 - lon1)**2
    return math.sqrt(f1 + f2)

def get_angle_asin(opposite_side:float, hypotenuse:float) -> float:
    """
    Calculate the angle between two sides.
    """
    return rad_to_deg(math.asin(opposite_side / hypotenuse))

def need_to_correct_route(location: _Location, robot_lat: float, robot_lon: float, compass: float) -> str:
    """
    Verify if the robot needs to correct the route.
    """

    # Error trying to read robot lat and lon
    if robot_lat >= 0 or robot_lon >= 0:
        return "error"

    # Setting the mission lat and lon to be relative to robot (0,0)
    mission_lat: float = location.latitude - robot_lat
    mission_lon: float = location.longitude - robot_lon

    # Setting robot to (0,0)
    robot_lat = 0
    robot_lon = 0
    
    H = dist_two_points(robot_lat, robot_lon, mission_lat, mission_lon)
    co = dist_two_points(robot_lat, robot_lon, robot_lat, mission_lon)
    Beta = 90 - get_angle_asin(co, H)

    margin = 10

    if mission_lon < 0:  # Quadrante baixo
        if mission_lat < 0:  # Quadrante esquerda
            Beta = Beta + 180

            if(compass <= Beta + margin and compass >= Beta - margin):
                return "forward"

            if compass >= Beta or compass <= Beta-180:
                return "left" #Virar esquerda
            return "right" #Virar direita
        else:  # Quadrante direita
            Beta = 180 - Beta

            if(compass <= Beta + margin and compass >= Beta - margin):
                return "forward"

            if compass <= Beta or compass >=  Beta+180:
                return "right" #Virar Direita
            return "left" #Virar esquerda
    else:  # Quadrante cima
        if mission_lat < 0:  # Quadrante esquerda
            dif = 360 - Beta
            if(compass >= dif - margin and compass <= dif + margin ):
                return "forward"
            elif((dif + margin > 360 and compass <= dif + margin - 360)):
                return "forward"

            if (compass >= 0 and compass <= 180-Beta) or (compass >= 360-Beta):
                return "left"
            return "right"
        else:  # Quadrante direita

            if((compass >= Beta and compass <= Beta + margin) or (compass >= 0 and compass >= Beta - margin and compass <= Beta + margin)):
                return "forward"
            elif(Beta - margin < 0 and compass >= 360 + (Beta - margin)):
                return "forward"

            if (compass <= 180+Beta and compass >= 90) or (compass > Beta and compass <= 90):
                return "left"
            return "right"