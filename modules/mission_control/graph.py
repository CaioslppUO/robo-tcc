import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from point import Point, Points

def plot(points: Points, robot: Point = None, closest_point: Point = None, correction_point: Point = None) -> None:
    fig = plt.figure()
    ax = plt.axes()

    # Maximized Window
    plt.get_current_fig_manager().window.attributes('-zoomed', True)

    # Axis labels
    plt.xlabel("LON")
    plt.ylabel("LAT")

    # X and Y axis on 0
    plt.axvline(x=0, c="red", label="x=0")
    plt.axhline(y=0, c="yellow", label="y=0")

    # Plotting mission points
    plt.scatter(x=points.get_longitudes(), y=points.get_latitudes())

    # Plotting mission direction
    ax.annotate("",
            xy=(points.get_longitudes()[-1], points.get_latitudes()[-1]),
            xytext=(0, 0),
            va="center",
            ha="right",
            arrowprops={"arrowstyle": "-|>", "lw": 0.5},
            label="Direção da missão")

    # Graph limits
    x_sup_limit = round(abs(max(points.get_longitudes(), key=abs))*1.1, 5)
    y_sup_limit = round(abs(max(points.get_latitudes(), key=abs))*1.1, 5)


    if(x_sup_limit == 0):
        x_sup_limit = y_sup_limit
    elif(y_sup_limit == 0):
        y_sup_limit = x_sup_limit

    plt.xlim(-x_sup_limit, x_sup_limit)
    plt.ylim(-y_sup_limit, y_sup_limit)

    # Plotting robot, closest point and correction point
    if(robot != None):
        plt.plot([robot.longitude], [robot.latitude], marker="o", markeredgecolor="green", markerfacecolor="green", label="Robô")
    
    if(closest_point != None):
        plt.plot([closest_point.longitude], [closest_point.latitude], marker="o", markeredgecolor="red", markerfacecolor="red", label="Ponto mais próximo do robô")
    
    if(correction_point != None):
        plt.plot([correction_point.longitude], [correction_point.latitude], marker="o", markeredgecolor="orange", markerfacecolor="orange", label="Ponto utilizado para correção")

    plt.legend(loc="upper left")
    plt.show()

def test() -> None:
    points = Points()
    points.add_point(0.0, 0.0)
    points.add_point(1.0, 1.0)
    points.add_point(2.0, 2.0)

    plot(points)

    points = Points()
    points.add_point(0.0, 0.0)
    points.add_point(-1.0, -1.0)
    points.add_point(-2.0, -2.0)

    plot(points)

    points = Points()
    points.add_point(0.0, 0.0)
    points.add_point(1.0, -1.0)
    points.add_point(2.0, -2.0)

    plot(points)

    points = Points()
    points.add_point(0.0, 0.0)
    points.add_point(-1.0, 1.0)
    points.add_point(-2.0, 2.0)

    plot(points)

#test()