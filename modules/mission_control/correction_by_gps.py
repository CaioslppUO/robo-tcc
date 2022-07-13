from turtle import width
from point2 import Points, Point
from auto_mode_calcs import calculate_y_by_line_equation, dist_two_points
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')

class GPSCorrection:
    def __init__(self, points_interval: float) -> None:
        self.points = Points()
        self.points_interval = points_interval

    def plot(
            self, vet_lon: "list[float]", vet_lat: "list[float]",
            slope: float, total_distance: float, total_points: int,
            robot_point: Point, closest_point: Point, correction_point: Point
        ) -> None:
        fig = plt.figure()
        ax = plt.axes()

        plt.scatter(vet_lon[0:-1], vet_lat[0:-1], label="Pontos da missão")
        #plt.arrow(vet_lon[-1], vet_lat[-1], self.points_interval, self.points_interval, width=self.points_interval/2, label="Direção da missão")
        
        ax.annotate("",
            xy=(vet_lon[-1], vet_lat[-1]),
            xytext=(0, 0),
            va="center",
            ha="right",
            arrowprops={"arrowstyle": "-|>", "lw": 0.5},
            label="Direção da missão")

        plt.xlabel("LON")
        plt.ylabel("LAT")

        plt.figtext(0.77, 0.8, "Intervalo: {:10.5f}".format(self.points_interval), fontsize=10)
        plt.figtext(0.77, 0.75, "Coeficiente Angular: {:10.5f}".format(slope), fontsize=10)
        plt.figtext(0.77, 0.70, "Distância Total: {:10.5f}".format(total_distance), fontsize=10)
        plt.figtext(0.77, 0.65, "Total de Pontos: {:10}".format(total_points), fontsize=10)
        plt.figtext(0.77, 0.60, "Ponto mais próximo do robô: {}".format(closest_point.id), fontsize=10)

        x_sup_limit = max(vet_lon)*1.5
        y_sup_limit = max(vet_lat)*1.5

        plt.xlim(-x_sup_limit, x_sup_limit)
        plt.ylim(-y_sup_limit, y_sup_limit)

        plt.plot([closest_point.longitude], [closest_point.latitude], marker="o", markeredgecolor="red", markerfacecolor="red", label="Ponto mais próximo do robô")
        plt.plot([robot_point.longitude], [robot_point.latitude], marker="o", markeredgecolor="green", markerfacecolor="green", label="Robô")
        plt.plot([correction_point.longitude], [correction_point.latitude], marker="o", markeredgecolor="orange", markerfacecolor="orange", label="Ponto utilizado para correção")
        
        plt.legend(loc="upper left")

        plt.axvline(x=0, c="red", label="x=0")
        plt.axhline(y=0, c="yellow", label="y=0")

        plt.show()

    def get_points_between(self, robot_lat: float, robot_lon: float, 
            mission_lat: float, mission_lon: float) -> "tuple[list[float], list[float], int, float]":
        vet_lat: "list[float]" = []
        vet_lon: "list[float]" = []

        dist = dist_two_points(robot_lat, robot_lon, mission_lat, mission_lon)
        n_of_points = int(dist / self.points_interval)

        longitudes = 0
        for _ in range(0, n_of_points):
            latitude, slope = calculate_y_by_line_equation(longitudes, mission_lon, mission_lat)

            vet_lon.append(longitudes)
            vet_lat.append(round(latitude, 6))

            if(dist_two_points(vet_lat[-1], longitudes, mission_lat, mission_lon) > dist):
                break

            if(mission_lat < 0):
                longitudes -= self.points_interval
            else:
                longitudes += self.points_interval

        return vet_lat, vet_lon, len(vet_lat), dist, slope

def test() -> None:
    points_interval = 0.00003

    original_points = [
        -25.43548, # Robot lat
        -54.59701, # Robot lon
        -25.43516, # Mission lat
        -54.59685 # Mission lon
    ]

    correct_points = [
        0, # Robot lat
        0, # Robot lon
        round(original_points[2] - original_points[0], 6), # Mission lat
        round(original_points[3] - original_points[1], 6) # Mission lon
    ]

    robot_position = Point(0.00012, 0.00003, 0)

    g = GPSCorrection(points_interval)

    # Calculating the points between
    vet_lat, vet_lon, n_of_points, total_distance, slope = g.get_points_between(
        correct_points[0],
        correct_points[1],
        correct_points[2],
        correct_points[3]
    )

    g.points.add_points(vet_lat, vet_lon)

    # Calculating closest point
    closest_point, correction_point = g.points.get_order_by_distance(robot_position)

    g.plot(vet_lon, vet_lat, slope, total_distance, n_of_points, robot_position, closest_point, correction_point)

test()