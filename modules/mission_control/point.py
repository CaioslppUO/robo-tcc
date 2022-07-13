from auto_mode_calcs import dist_two_points
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')

class Point:
    def __init__(self,latitude:float,longitude:float,id:int) -> None:
        self.latitude:float = latitude
        self.longitude:float = longitude
        self.id:int = id
    
class Points:
    def __init__(self) -> None:
        self.points:"list[Point]" = []

    def add_point(self,latitude:float,longitude:float) -> None:
        self.points.append(Point(latitude,longitude,len(self.points)))
    
    def get_order_by_distance(self,latitude:float,longitude:float) -> Point:
        dist:"list[float]" = []
        minDist = -1
        minIndex= 0
        for i in self.points:
            dist.append(dist_two_points(latitude,longitude,i.latitude,i.longitude))
        for i in range(len(dist)):
            if minDist == -1 or dist[i] < minDist:
                minDist = dist[i]
                minIndex = i
        return self.points[minIndex]

    
def get_y(x: float, y2: float, x2: float) -> float:
    return y2/x2 * x

def generate_info_log(
        robot_lat_i: float, robot_lon_i: float, 
        mission_lat_i: float, mission_lon_i: float,
        robot_lat_f: float, robot_lon_f: float,
        mission_lat_f: float, mission_lon_f: float,
        distance_between_points: float, n_of_points: int, interval: float) -> None:
    print("-" * 50)
    print("            ** Dados da missão ** \n")
    print("Ponto inicial robô:     ({:10.6f}, {:10.6f})".format(robot_lat_i, robot_lon_i))
    print("Ponto inicial missão:   ({:10.6f}, {:10.6f})".format(mission_lat_i, mission_lon_i))
    print("Ponto corrigido robô:   ({:10.6f}, {:10.6f})".format(robot_lat_f, robot_lon_f))
    print("Ponto corrigido missão: ({:10.6f}, {:10.6f})".format(mission_lat_f, mission_lon_f))
    print("\n")
    print("_" * 50)
    print("  ** Dados dos pontos e correções por GPS ** \n")
    print("Intervalo entre os pontos: {:.6f}".format(interval))
    print("Distância entre os dois pontos: {:.6f}".format(distance_between_points))
    print("Número de pontos: {}".format(n_of_points))
    print("-" * 50)

def plot(x_vet: "list[float]", y_vet: "list[float]", interval: float, n_of_points: int, robot_now_lat: float, robot_now_lon: float, selected_5_point_lat: float, selected_5_point_lon: float, selected_point_lat: float, selected_point_lon: float) -> None:
    fig = plt.figure()
    ax = plt.axes()

    #plt.plot(xes, vet)
    plt.scatter(y_vet, x_vet)

    plt.xlabel("LON")
    plt.ylabel("LAT")

    #plt.xlim(-interval*n_of_points/4, x_vet[-1] + interval*n_of_points/4)
    #plt.ylim(-interval*n_of_points/4, y_vet[-1] + interval*n_of_points/4)

    plt.axvline(x=0, c="red", label="x=0")
    plt.axhline(y=0, c="yellow", label="y=0")


    plt.plot([robot_now_lon], [robot_now_lat], marker="o", markeredgecolor="green", markerfacecolor="green")
    plt.plot([selected_point_lon], [selected_point_lat], marker="o", markeredgecolor="red", markerfacecolor="red")
    plt.plot([selected_5_point_lon], [selected_5_point_lat], marker="o", markeredgecolor="orange", markerfacecolor="orange")


    plt.show()

def get_points_between(interval: float, 
                        robot_lat: float, robot_lon: float, 
                        mission_lat: float, mission_lon: float) -> "tuple[list[float], list[float], int, float]":
    vet_x: "list[float]" = []
    vet_y: "list[float]" = []

    dist = dist_two_points(robot_lat, robot_lon, mission_lat, mission_lon)
    n_of_points = int(dist / interval)

    x = 0
    for i in range(0, n_of_points):
        vet_x.append(x)
        vet_y.append(round(get_y(x, mission_lon, mission_lat), 6))
        if(dist_two_points(x, vet_y[-1], mission_lat, mission_lon) > dist):
            break
        if(mission_lat < 0):
            x -= interval
        else:
            x += interval

    return vet_x, vet_y, len(vet_x), dist

def should_turn_to(p1_lat: float, p1_lon: float, p2_lat: float, p2_lon: float) -> str:
    if(p2_lat > p1_lat):
        print("reto")
    elif(p2_lat < p1_lat):
        print("reto")
    else:
        print("Subir/Descer: nenhum")
    
    if(p2_lon > p1_lon):
        print("direita")
    elif(p2_lon < p1_lon):
        print("esquerda")
    else:
        print("Esquerda/Direita: nenhum")

def test_quad_1() -> None:
    space_between_points = 0.000003

    robot_lat_original = -25.43548
    robot_lon_original = -54.59701

    mission_lat_original = -25.43532 
    mission_lon_original = -54.59694

    mission_lat = round(mission_lat_original - robot_lat_original, 6)
    mission_lon = round(mission_lon_original - robot_lon_original, 6)

    robot_lat, robot_lon = 0, 0

    vet_x, vet_y, n_of_points, dist_between = get_points_between(space_between_points, robot_lat, robot_lon, mission_lat, mission_lon)

    generate_info_log(robot_lat_original, robot_lon_original, mission_lat_original, mission_lon_original, 
                    robot_lat, robot_lon, mission_lat, mission_lon, dist_between, n_of_points, space_between_points)

    # Mission points
    p = Points()
    for i in range(0, len(vet_x)):
        p.add_point(vet_x[i], vet_y[i])

    # Simulating Robot Position
    # Simulating Robot Position
    # Em cima da reta
    r_la, r_lo = 0.00012, 0.00003
    # A baixo da reta
    #r_la, r_lo = 0.00004, 0.00003
    vet_x.append(r_la)
    vet_y.append(r_lo)

    print("Ponto do robô: ({:10.6f}, {:10.6f})".format(r_la, r_lo))
    menor_ponto = p.get_order_by_distance(r_la, r_lo)
    print("Menor Ponto: {}".format(menor_ponto.id))
    print("Deveria estar em: ({:10.6f}, {:10.6f})".format(menor_ponto.latitude, menor_ponto.longitude))
    proximo_5: Point = p.points[menor_ponto.id + 5]
    print("Corrigir para o proximo (5pontos): ({:10.6f}, {:10.6f})".format(proximo_5.latitude, proximo_5.longitude))

    should_turn_to(r_la, r_lo, proximo_5.latitude, proximo_5.longitude)
    plot(vet_x, vet_y, space_between_points, n_of_points, r_la, r_lo, proximo_5.latitude, proximo_5.longitude, menor_ponto.latitude, menor_ponto.longitude)

def test_quad_4() -> None:
    space_between_points = 0.000003

    robot_lat_original = -25.43548
    robot_lon_original = -54.59701

    mission_lat_original = -25.43512
    mission_lon_original = -54.59712

    mission_lat = round(mission_lat_original - robot_lat_original, 6)
    mission_lon = round(mission_lon_original - robot_lon_original, 6)

    robot_lat, robot_lon = 0, 0

    vet_x, vet_y, n_of_points, dist_between = get_points_between(space_between_points, robot_lat, robot_lon, mission_lat, mission_lon)

    generate_info_log(robot_lat_original, robot_lon_original, mission_lat_original, mission_lon_original, 
                    robot_lat, robot_lon, mission_lat, mission_lon, dist_between, n_of_points,space_between_points)

    # Mission points
    p = Points()
    for i in range(0, len(vet_x)):
        p.add_point(vet_x[i], vet_y[i])

    # Simulating Robot Position
    # Em cima da reta
    r_la, r_lo = 0.00012, -0.00002
    # A baixo da reta
    #r_la, r_lo = 0.00012, -0.00009
    
    vet_x.append(r_la)
    vet_y.append(r_lo)

    print("Ponto do robô: ({:10.6f}, {:10.6f})".format(r_la, r_lo))
    menor_ponto = p.get_order_by_distance(r_la, r_lo)
    print("Menor Ponto: {}".format(menor_ponto.id))
    print("Deveria estar em: ({:10.6f}, {:10.6f})".format(menor_ponto.latitude, menor_ponto.longitude))
    proximo_5: Point = p.points[menor_ponto.id + 5]
    print("Corrigir para o proximo (5pontos): ({:10.6f}, {:10.6f})".format(proximo_5.latitude, proximo_5.longitude))

    should_turn_to(r_la, r_lo, proximo_5.latitude, proximo_5.longitude)
    
    plot(vet_x, vet_y, space_between_points, n_of_points, r_la, r_lo, proximo_5.latitude, proximo_5.longitude, menor_ponto.latitude, menor_ponto.longitude)

def test_quad_2() -> None:
    space_between_points = 0.000003

    robot_lat_original = -25.43548
    robot_lon_original = -54.59701

    mission_lat_original = -25.43560
    mission_lon_original = -54.59660

    mission_lat = round(mission_lat_original - robot_lat_original, 6)
    mission_lon = round(mission_lon_original - robot_lon_original, 6)

    robot_lat, robot_lon = 0, 0

    vet_x, vet_y, n_of_points, dist_between = get_points_between(space_between_points, robot_lat, robot_lon, mission_lat, mission_lon)

    generate_info_log(robot_lat_original, robot_lon_original, mission_lat_original, mission_lon_original, 
                    robot_lat, robot_lon, mission_lat, mission_lon, dist_between, n_of_points, space_between_points)

    # Mission points
    p = Points()
    for i in range(0, len(vet_x)):
        p.add_point(vet_x[i], vet_y[i])

    # Simulating Robot Position
    # Em cima da reta
    #r_la, r_lo = -0.00001, 0.00005
    # A baixo da reta
    r_la, r_lo = -0.00009, 0.00014
    vet_x.append(r_la)
    vet_y.append(r_lo)

    print("Ponto do robô: ({:10.6f}, {:10.6f})".format(r_la, r_lo))
    menor_ponto = p.get_order_by_distance(r_la, r_lo)
    print("Menor Ponto: {}".format(menor_ponto.id))
    print("Deveria estar em: ({:10.6f}, {:10.6f})".format(menor_ponto.latitude, menor_ponto.longitude))
    proximo_5: Point = p.points[menor_ponto.id + 5]
    print("Corrigir para o proximo (5pontos): ({:10.6f}, {:10.6f})".format(proximo_5.latitude, proximo_5.longitude))

    should_turn_to(r_la, r_lo, proximo_5.latitude, proximo_5.longitude)
    
    plot(vet_x, vet_y, space_between_points, n_of_points, r_la, r_lo, proximo_5.latitude, proximo_5.longitude, menor_ponto.latitude, menor_ponto.longitude)

def test_quad_3() -> None:
    space_between_points = 0.000003

    robot_lat_original = -25.43548
    robot_lon_original = -54.59701

    mission_lat_original = -25.43560
    mission_lon_original = -54.59709

    mission_lat = round(mission_lat_original - robot_lat_original, 6)
    mission_lon = round(mission_lon_original - robot_lon_original, 6)

    robot_lat, robot_lon = 0, 0

    vet_x, vet_y, n_of_points, dist_between = get_points_between(space_between_points, robot_lat, robot_lon, mission_lat, mission_lon)

    generate_info_log(robot_lat_original, robot_lon_original, mission_lat_original, mission_lon_original, 
                    robot_lat, robot_lon, mission_lat, mission_lon, dist_between, n_of_points, space_between_points)

    # Mission points
    p = Points()
    for i in range(0, len(vet_x)):
        p.add_point(vet_x[i], vet_y[i])

    # Simulating Robot Position
    # Em cima da reta
    #r_la, r_lo = -0.00012, -0.00007
    # A baixo da reta
    r_la, r_lo = -0.00002, -0.00002
    vet_x.append(r_la)
    vet_y.append(r_lo)

    print("Ponto do robô: ({:10.6f}, {:10.6f})".format(r_la, r_lo))
    menor_ponto = p.get_order_by_distance(r_la, r_lo)
    print("Menor Ponto: {}".format(menor_ponto.id))
    print("Deveria estar em: ({:10.6f}, {:10.6f})".format(menor_ponto.latitude, menor_ponto.longitude))
    proximo_5: Point = p.points[menor_ponto.id + 5]
    print("Corrigir para o proximo (5pontos): ({:10.6f}, {:10.6f})".format(proximo_5.latitude, proximo_5.longitude))

    should_turn_to(r_la, r_lo, proximo_5.latitude, proximo_5.longitude)
    
    plot(vet_x, vet_y, space_between_points, n_of_points, r_la, r_lo, proximo_5.latitude, proximo_5.longitude, menor_ponto.latitude, menor_ponto.longitude)

#test_quad_1()
#test_quad_2()
#test_quad_4()
test_quad_3()