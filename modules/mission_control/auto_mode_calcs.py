import math

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