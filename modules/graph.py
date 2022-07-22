from matplotlib.markers import MarkerStyle
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('seaborn-whitegrid')
from geometry import Line,Point

class Graph:
    def __init__(self,interval:int = 2000):
        self.interval:int = interval
        self.straight_from_mission_lon:"list[float]"  = []
        self.straight_from_mission_lat:"list[float]"  = []
        self.robot_position:"tuple[float,float]" = None
        self.closest_point:"tuple[float,float]" = None
        self.robot_route_lat:"list[float]"  = []
        self.robot_route_lon:"list[float]"  = []
        self.correction_direction:"str" = None
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
        plt.xlim(min(self.straight_from_mission_lon), max(self.straight_from_mission_lon))
        plt.ylim(min(self.straight_from_mission_lat), max(self.straight_from_mission_lat))
        plt.xlabel("LON")
        plt.ylabel("LAT")

        plt.cla()

        # Plot mission line
        plt.plot(self.straight_from_mission_lon, self.straight_from_mission_lat, marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Missão')

        #Plot robot route
        if(len(self.robot_route_lon) > 0):
            plt.plot(self.robot_route_lon, self.robot_route_lat, marker='o', markerfacecolor='green', markersize=6, color='green', label='Rota do robô')

        #Plot robot position
        if(self.robot_position is not None):
            plt.plot(self.robot_position[0], self.robot_position[1], marker='o', markerfacecolor='red', markersize=6, color='red', label='Robô')

        #Plot closest point
        if(self.closest_point is not None):
            plt.plot(self.closest_point[0], self.closest_point[1], marker='o', markerfacecolor='black', markersize=6, color='black', label='Ponto mais próximo')

        if(self.correction_direction is not None): # Add correction_direction to the legend
            plt.plot([0], [0], markersize=0.1 , marker="o", markeredgecolor="green", markerfacecolor="blue", label="Direção de correção: {}".format(self.correction_direction))
            # Plotting correction direction
            plt.annotate("",
                xy=(0, self.straight_from_mission_lat[-1]),
                xytext=(0, self.straight_from_mission_lat[-1]),
                va="center",
                ha="right",
                arrowprops={"arrowstyle": "-", "lw": 0},
                label="Direção de correção: {}".format(self.correction_direction))

        #Plotting mission direction
        plt.annotate("",
            xy=(self.straight_from_mission_lon[-1], self.straight_from_mission_lat[-1]),
            xytext=(self.straight_from_mission_lon[0], self.straight_from_mission_lat[0]),
            va="center",
            ha="right",
            arrowprops={"arrowstyle": "-|>", "lw": 0.5})

        plt.legend(loc="upper left")
        plt.tight_layout()
        

    def set_corretion_direction_legend(self,direction:str):
        """
        Set the correction direction legend
        """
        self.correction_direction = direction

    def set_straight_from_mission(self,points:Line):
        """
        Store mission line points
        """
        if(len(self.straight_from_mission_lon) > 0):
            self.straight_from_mission_lon.clear()
            self.straight_from_mission_lat.clear()
        for point in points.get_points():
            self.straight_from_mission_lat.append(point.get_point()[0])
            self.straight_from_mission_lon.append(point.get_point()[1])

    def new_position_robot(self,position:"tuple[float,float]"):
        """
        Store the new robot position and update route taken by robot
        """
        self.robot_route_lon.append(position[0])
        self.robot_route_lat.append(position[1])
        self.robot_position = position;

def test_func():
    test = Graph()
    lineTest = Line(10)
    lineTest.add(Point(0,0))
    lineTest.add(Point(1,1))
    lineTest.add(Point(2,2))
    lineTest.add(Point(3,3))
    test.set_straight_from_mission(lineTest)
    test.new_position_robot((3,1))
    test.run()

# test_func()