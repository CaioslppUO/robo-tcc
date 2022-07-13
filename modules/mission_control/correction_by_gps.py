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
            robot_point: Point, closest_point: Point, correction_point: Point,
            title: str
        ) -> None:
        fig = plt.figure()
        ax = plt.axes()

        plt.get_current_fig_manager().window.attributes('-zoomed', True)

        plt.subplots_adjust(left=0.3, right=0.95, top=0.9, bottom=0.1)

        plt.title(title)

        plt.scatter(vet_lon[0:-1], vet_lat[0:-1], label="Pontos da missão")
        
        ax.annotate("",
            xy=(vet_lon[-1], vet_lat[-1]),
            xytext=(0, 0),
            va="center",
            ha="right",
            arrowprops={"arrowstyle": "-|>", "lw": 0.5},
            label="Direção da missão")

        plt.xlabel("LON")
        plt.ylabel("LAT")

        x_sup_limit = abs(max(vet_lon, key=abs))*1.5
        y_sup_limit = abs(max(vet_lat, key=abs))*1.5

        plt.figtext(x_sup_limit, y_sup_limit+0.80, "Intervalo: {:10.5f}".format(self.points_interval), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.75, "Coeficiente Angular: {:10.5f}".format(slope), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.70, "Distância Total: {:10.5f}".format(total_distance), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.65, "Total de Pontos: {:10}".format(total_points), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.60, "Ponto mais próximo do robô: {}".format(closest_point.id), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.55, "Ponto inicial: ({:10.5f}, {:10.5f})".format(vet_lat[0], vet_lon[0]), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.50, "Ponto final: ({:10.5f}, {:10.5f})".format(vet_lat[-1], vet_lon[-1]), fontsize=10)
        plt.figtext(x_sup_limit, y_sup_limit+0.45, "Localização atual do robô: ({:10.5f}, {:10.5f})".format(robot_point.latitude, robot_point.longitude), fontsize=10)



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
                #break
                pass

            if(mission_lat < 0):
                longitudes -= self.points_interval
            else:
                longitudes += self.points_interval

        return vet_lat, vet_lon, len(vet_lat), dist, slope

def test(points_interval: float, mission_start: Point, mission_end: Point, robot_position: Point, title: str) -> None:
    g = GPSCorrection(points_interval)

    # Calculating the points between start and end
    vet_lat, vet_lon, n_of_points, total_distance, slope = g.get_points_between(
        mission_start.latitude,
        mission_start.longitude,
        mission_end.latitude,
        mission_end.longitude
    )
    
    # Adding calculated points to points array
    g.points.add_points(vet_lat, vet_lon)

    # Calculating closest point
    closest_point, correction_point = g.points.get_order_by_distance(robot_position)

    # Ploting graphic
    g.plot(vet_lon, vet_lat, slope, total_distance, n_of_points, robot_position, closest_point, correction_point, title)

def test_quadrant(points_interval: float, original_mission_points: "tuple[float, float, float, float]",
        robot_position_1: Point, robot_position_2: Point, quadrant: str) -> None:
    mission_start = Point(0, 0, 0)
    mission_end = Point(round(original_mission_points[2] - original_mission_points[0], 6), round(original_mission_points[3] - original_mission_points[1], 6), 0)

    test(points_interval, mission_start, mission_end, robot_position_1, "Simulação 1 - Quadrante {} - robô acima da linha".format(quadrant))
    
    test(points_interval, mission_start, mission_end, robot_position_2, "Simulação 2 - Quadrante {} - robô abaixo da linha".format(quadrant))


def test_all_quadrants() -> None:
    points_interval = 0.00003

    # Up Right
    original_points = [
        -25.43548, -54.59701, # Start (lat, lon)
        -25.43516, -54.59685 # End (lat, lon)
    ]
    robot_position_1 = Point(0.00020, 0.00003, 0)
    robot_position_2 = Point(0.00008, 0.00009, 0)
    test_quadrant(points_interval, original_points, robot_position_1, robot_position_2, "superior da direita")

    # Up Left
    original_points = [
        -25.43548, -54.59701, # Start (lat, lon)
        -25.43555, -54.59685 # End (lat, lon)
    ]
    robot_position_1 = Point(0.00002, -0.00003, 0)
    robot_position_2 = Point(0.00001, -0.00009, 0)
    test_quadrant(points_interval, original_points, robot_position_1, robot_position_2, "superior da esquerda")


test_all_quadrants()