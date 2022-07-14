import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from point import Point, Points

def plot(points: Points) -> None:
    fig = plt.figure()
    ax = plt.axes()

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
    x_sup_limit = abs(max(points.get_longitudes(), key=abs))*1.1
    y_sup_limit = abs(max(points.get_latitudes(), key=abs))*1.1

    if(x_sup_limit == 0):
        x_sup_limit = y_sup_limit
    elif(y_sup_limit == 0):
        y_sup_limit = x_sup_limit

    plt.xlim(-x_sup_limit, x_sup_limit)
    plt.ylim(-y_sup_limit, y_sup_limit)

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