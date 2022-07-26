import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import os

plt.style.use('seaborn-whitegrid')
from geometry import Point

class GraphData:
    def __init__(self):
        self.straight_from_mission_lon:"list[float]"  = []
        self.straight_from_mission_lat:"list[float]"  = []
        self.robot_position:"tuple[float,float]" = None
        self.closest_point:"tuple[float,float]" = None
        self.robot_route_lat:"list[float]"  = []
        self.robot_route_lon:"list[float]"  = []
        self.correction_direction:"str" = None
        self.fieldnames = ["straight_from_mission_lon", "straight_from_mission_lat", "robot_position", "closest_point", "robot_route_lat", "robot_route_lon", "correction_direction"]
        os.system("python3 run_graph.py&")

    def __write_to_json(self):
        """
        Write the data to a csv file
        """
        with open('graph.json', 'w') as outfile:
            info = {
                "straight_from_mission_lon": self.straight_from_mission_lon,
                "straight_from_mission_lat": self.straight_from_mission_lat,
                "robot_position": self.robot_position,
                "closest_point": self.closest_point,
                "robot_route_lat": self.robot_route_lat,
                "robot_route_lon": self.robot_route_lon,
                "correction_direction": self.correction_direction
            }
            outfile.write(json.dumps(info))
            print(json.dumps(info))

    def set_corretion_direction_legend(self,direction:str):
        """
        Set the correction direction legend
        """
        self.correction_direction = direction
        self.__write_to_json()

    def set_straight_from_mission(self,points:"list[Point]"):
        """
        Store mission line points
        """
        if(len(self.straight_from_mission_lon) > 0):
            self.straight_from_mission_lon.clear()
            self.straight_from_mission_lat.clear()
        for point in points:
            self.straight_from_mission_lat.append(point.get_point()[0])
            self.straight_from_mission_lon.append(point.get_point()[1])
        self.__write_to_json()

    def new_position_robot(self,position:"tuple[float,float]"):
        """
        Store the new robot position and update route taken by robot
        """
        self.robot_route_lon.append(position[0])
        self.robot_route_lat.append(position[1])
        self.robot_position = position;
        self.__write_to_json()

    def clean_robot_route(self):
        """
        Reset the robot route
        """
        self.robot_route_lon.clear()
        self.robot_route_lat.clear()
        self.__write_to_json()

class Graph:
    def __init__(self,interval:int = 2000):
        self.interval:int = interval
        
        try:
            plt.get_current_fig_manager().window.attributes('-zoomed', True)
        except:
            pass

    def run(self):
        """
        Run the graph
        """
        self.animation = FuncAnimation(plt.gcf(), self.update, interval=self.interval)
        plt.show()

    def update(self,frame):
        """
        Update the graph
        """
        try:
            with open('graph.json') as json_file:
                data = json.load(json_file)
        except:
            data = None
        plt.cla()
        plt.xlabel("LON")
        plt.ylabel("LAT")
        if data is not None:
            if(len(data["straight_from_mission_lon"]) > 0):
                plt.xlim(min(data["straight_from_mission_lon"])-0.00002, max(data["straight_from_mission_lon"])+0.00002)
                plt.ylim(min(data["straight_from_mission_lat"])-0.00002, max(data["straight_from_mission_lat"])+0.00002)
                
                # Plot mission line
                plt.plot(data["straight_from_mission_lon"], data["straight_from_mission_lat"], marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Missão')
                #Plotting mission direction
                plt.annotate("",
                    xy=(data["straight_from_mission_lon"][-1], data["straight_from_mission_lat"][-1]),
                    xytext=(data["straight_from_mission_lon"][0], data["straight_from_mission_lat"][0]),
                    va="center",
                    ha="right",
                    arrowprops={"arrowstyle": "-|>", "lw": 0.5})

            #Plot robot route
            if(len(data["robot_route_lon"]) > 0):
                plt.plot(data["robot_route_lon"], data["robot_route_lat"], marker='o', markerfacecolor='green', markersize=6, color='green', label='Rota do robô')

            #Plot robot position
            if(data["robot_position"] is not None):
                plt.plot(data["robot_position"][0], data["robot_position"][1], marker='o', markerfacecolor='red', markersize=6, color='red', label='Robô')

            #Plot closest point
            if(data["closest_point"] is not None):
                plt.plot(data["closest_point"][0], data["closest_point"][1], marker='o', markerfacecolor='black', markersize=6, color='black', label='Ponto mais próximo')

            if(data["correction_direction"] is not None): # Add correction_direction to the legend
                plt.plot([0], [0], markersize=0.1 , marker="o", markeredgecolor="green", markerfacecolor="blue", label="Direção de correção: {}".format(self.correction_direction))
                # Plotting correction direction
                plt.annotate("",
                    xy=(0, data["straight_from_mission_lat"][-1]),
                    xytext=(0, data["straight_from_mission_lat"][-1]),
                    va="center",
                    ha="right",
                    arrowprops={"arrowstyle": "-", "lw": 0},
                    label="Direção de correção: {}".format(data["correction_direction"]))

        plt.legend(loc="upper left")
        plt.tight_layout()


def write_graph():
    """
    Exemplo de como escrever um gráfico
    """
    gpData = GraphData()
    lineTest = []
    lineTest.append(Point(0,0))
    lineTest.append(Point(1,1))
    lineTest.append(Point(2,2))
    lineTest.append(Point(3,3))
    gpData.set_straight_from_mission(lineTest)
    gpData.new_position_robot((3,1))
    gpData.new_position_robot((3,2))
    gpData.new_position_robot((2,2))

# write_graph()