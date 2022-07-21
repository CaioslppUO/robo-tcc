import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from point import Point
from points import Points


fig = plt.figure()
ax = plt.axes()

def plot(points: Points, robot: Point = None, closest_point: Point = None, correction_point: Point = None, correction_direction: str = None, robot_points: Points = None, mission_quadrant: int = None) -> None:
    global fig,ax

    # Maximized Window
    # plt.get_current_fig_manager().window.attributes('-zoomed', True)

    # Axis labels
    plt.xlabel("LON")
    plt.ylabel("LAT")

    # X and Y axis on 0
    #plt.axvline(x=0, c="red", label="x=0")
    #plt.axhline(y=0, c="yellow", label="y=0")

    # Plotting mission points
    plt.scatter(x=points.get_longitudes(), y=points.get_latitudes())

    # Plotting mission direction
    ax.annotate("",
            xy=(points.get_longitudes()[-1], points.get_latitudes()[-1]),
            xytext=(points.get_longitudes()[0], points.get_latitudes()[0]),
            va="center",
            ha="right",
            arrowprops={"arrowstyle": "-|>", "lw": 0.5},
            label="Direção da missão")

    ## Plotting correction direction
    #ax.annotate("direção de correção: {}".format(correction_direction),
    #        xy=(0, points.get_latitudes()[-1]),
    #        xytext=(0, points.get_latitudes()[-1]),
    #        va="center",
    #        ha="right",
    #        arrowprops={"arrowstyle": "-", "lw": 0},
    #        label="-")

    # Graph limits
    #x_sup_limit = round(abs(max(points.get_longitudes(), key=abs))*1.1, 5)
    #y_sup_limit = round(abs(max(points.get_latitudes(), key=abs))*1.1, 5)
#
#
    #if(x_sup_limit == 0):
    #    x_sup_limit = y_sup_limit
    #elif(y_sup_limit == 0):
    #    y_sup_limit = x_sup_limit

    min_x = min([points.get_longitudes()[0], points.get_longitudes()[-1]])
    max_x = max([points.get_longitudes()[0], points.get_longitudes()[-1]])

    min_y = min([points.get_latitudes()[0], points.get_latitudes()[-1]])
    max_y = max([points.get_latitudes()[0], points.get_latitudes()[-1]])
    plt.xlim(min_x - 0.0000002, max_x + 0.0000002)
    plt.ylim(min_y - 0.0000002, max_y + 0.0000002)

    x_values: "list[float]" = []
    for l in points.get_longitudes():
        x_values.append(l)
    if(robot != None):
        x_values.append(robot.longitude)

    y_values: "list[float]" = []
    for l in points.get_latitudes():
        y_values.append(l)
    if(robot != None):
        y_values.append(robot.latitude)

    plt.xticks(x_values)
    plt.yticks(y_values)
    if(closest_point != None):
        plt.plot([closest_point.longitude], [closest_point.latitude], marker="o", markeredgecolor="red", markerfacecolor="red", label="Ponto mais próximo do robô")
    
    if(correction_point != None):
        plt.plot([correction_point.longitude], [correction_point.latitude], marker="o", markeredgecolor="orange", markerfacecolor="orange", label="Ponto utilizado para correção")

    if(correction_direction != None): # Add correction_direction to the legend
        plt.plot([0], [0], markersize=0.1 , marker="o", markeredgecolor="green", markerfacecolor="green", label="Direção de correção: {}".format(correction_direction))

    # Plotting robot, closest point and correction point
    if(robot != None and robot_points == None):
        plt.plot([robot.longitude], [robot.latitude], marker="o", markeredgecolor="green", markerfacecolor="green", label="Robô")
    elif(robot_points != None and robot == None):
        for r_point in robot_points.get_points():
            plt.plot([r_point.longitude], [r_point.latitude], marker="o", markeredgecolor="green", markerfacecolor="green")



    if(mission_quadrant != None):
        if(mission_quadrant == 1):
            plt.legend(loc="upper left")
        elif(mission_quadrant == 2):
            plt.legend(loc="upper right")
        elif(mission_quadrant == 3):
            plt.legend(loc="upper left")
        elif(mission_quadrant == 4):
            plt.legend(loc="upper right")
    else:
        plt.legend(loc="upper left")

    fig.show()
    plt.pause(2)




def test() -> None:
    points = Points(10)
    points.add_point(0.0, 0.0)
    points.add_point(1.0, 1.0)
    points.add_point(2.0, 2.0)
    plot(points)

    points = Points(10)
    points.add_point(0.0, 0.0)
    points.add_point(-1.0, -1.0)
    points.add_point(-2.0, -2.0)
    plot(points)

    points = Points(10)
    points.add_point(0.0, 0.0)
    points.add_point(1.0, -1.0)
    points.add_point(2.0, -2.0)
    plot(points)

    points = Points(10)
    points.add_point(0.0, 0.0)
    points.add_point(-1.0, 1.0)
    points.add_point(-2.0, 2.0)
    plot(points)

test()