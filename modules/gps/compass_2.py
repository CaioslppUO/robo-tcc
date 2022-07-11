import math
import matplotlib.pyplot as plt


def deg_to_rad(ang: float) -> float:
    return math.radians(ang)

def rad_to_deg(ang: float) -> float:
    return ang * 180 / math.pi

def calculate_compass(angle: float, quadrant: int) -> int:
    """
    Calculate the compass of the robot in the North diretcion.
    """
    compass = -1
    angle = abs(angle)
    if(quadrant == 1):
        compass = angle
    elif(quadrant == 2):
        compass = 90 + angle
    elif(quadrant == 3):
        compass = 180 + angle
    elif(quadrant == 4):
        compass = 270 + angle
    return compass

def get_angle(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Calculate the angle between two points.
    """

    # Updating axis
    x1, y1, x2, y2 = round(x1, 6), round(y1, 6), round(x2, 6), round(y2, 6)
    x2, y2 = round(x2 - x1, 6), round(y2 - y1, 6)
    x1, y1 = 0, 0
    print("{:.6f}, {:.6f}, {:.6f}, {:.6f}".format(x1, y1, x2, y2))

    dx = x2 - x1
    dy = y2 - y1
    angle = 50.0

    if(dx != 0):
        angular_coeficient = dy / dx
        angle = round(math.atan(angular_coeficient), 6)
    else:
        angle = deg_to_rad(180.0)
        print("Caiu aqui para " + str(x1) + " " + str(y1) + " " + str(x2) + " " + str(y2))

    quadrant = -1

    if(dx < 0 and dy < 0):
        quadrant = 3
    elif(dx < 0 and dy >= 0):
        quadrant = 4
    elif(dx > 0 and dy < 0):
        quadrant = 2
    elif(dx > 0 and dy >= 0):
        quadrant = 1
    elif(dx == 0 and y2 >= y1):
        quadrant = -1
    elif(dx == 0 and y2 < y1):
        quadrant = -1

    return abs(rad_to_deg(angle)), quadrant, calculate_compass(rad_to_deg(angle), quadrant)



p3_x, p3_y = -25.437550, -54.596213
p4_x, p4_y = -25.437612, -54.595972
plt.scatter([p3_x, p4_x], [p3_y, p4_y])
# naming the x axis
plt.xlabel('x - axis')
# naming the y axis
plt.ylabel('y - axis')
  
# giving a title to my graph
plt.title('My first graph!')
  
# function to show the plot
plt.show()