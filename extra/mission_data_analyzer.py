"""
@package mission_data_analyzer.
Able to read mission_log.json files and plot what happened to the robot.
"""

import json, sys, traceback, time

try:
    import matplotlib.pyplot as plt
except:
    print(traceback.format_exc())

try:
    from agrobot.msg import Coords
    import rospy
    from std_msgs.msg import String
    rospy.init_node("mission_data_analyzer", anonymous=True)
    pub = rospy.Publisher("/gps", Coords, queue_size=10)
    pub_stop = rospy.Publisher("/stop_mission", String, queue_size=10)
except:
    print(traceback.format_exc())

class Mission:
    def __init__(self, mission_name: str, mission_line: "tuple[tuple[float, float], tuple[float, float]]", mission_points: "list[tuple[float, float]]") -> None:
        self.name = mission_name
        self.line = mission_line
        self.points = mission_points
        
class Correction:
    def __init__(self, correction_line: "tuple[tuple[float, float], tuple[float, float]]", correction_direction: str, error: float, chooser: int) -> None:
        self.line = correction_line
        self.direction = correction_direction
        self.error = error
        self.chooser = chooser

class Robot:
    def __init__(self, robot_pos: "tuple[float, float]", robot_direction_line: "tuple[tuple[float, float], tuple[float, float]]") -> None:
        self.robot_pos = robot_pos
        self.direction_line = robot_direction_line

class MissionRecord:
    def __init__(self, iteration: int, mission: Mission, correction: Correction, robot: Robot) -> None:
        self.iteration = iteration
        self.mission = mission
        self.correction = correction
        self.robot = robot

class MissionDataAnalyzer:
    def __init__(self, logs_file_path: str) -> None:
        self.logs_file = logs_file_path
        self.records: "list[MissionRecord]" = []
        self.total_records = 0
        self.__organize_file()
        self.__load_data()

    def __organize_file(self) -> None:
        data = None
        with open(self.logs_file, "r") as f:
            data = json.load(f)
            for entry in data:
                # Lines Transformations
                if("->" in entry["Robot Direction"]):
                    entry["Robot Direction"] = (entry["Robot Direction"].split("->")[0], entry["Robot Direction"].split("->")[1])
                if("->" in entry["Correction Line"]):
                    entry["Correction Line"] = (entry["Correction Line"].split("->")[0], entry["Correction Line"].split("->")[1])
                if("->" in entry["Mission Line"]):
                    entry["Mission Line"] = (entry["Mission Line"].split("->")[0], entry["Mission Line"].split("->")[1])
    
        with open(self.logs_file, "w") as f:
            json.dump(data, f, indent=4, separators=(',', ': '))

    def __load_data(self) -> None:
        with open(self.logs_file, "r") as f:
            data = json.load(f)
            i = 0
            self.total_records = 0
            for entry in data:
                # Mission Data
                try:
                    mission_name = entry["Mission Name"]
                    mission_line = entry["Mission Line"]
                    mission_line = [(mission_line[0].split(",")), (mission_line[1].split(","))]
                    mission_line = [(float(mission_line[0][0].split("(")[1]), float(mission_line[0][1].split(")")[0])), (float(mission_line[1][0].split("(")[1]), float(mission_line[1][1].split(")")[0]))]
                    mission_points = entry["Mission Points"]
                    mission = Mission(mission_name, mission_line, mission_points)
                except:
                    mission = Mission(None, None, None)

                # Iteration
                try:
                    iteration = entry["Iteration"]
                except:
                    iteration = None

                # Robot Data
                try:
                    robot_pos = entry["Robot Location"]
                    robot_pos = (robot_pos.split(",")[0], robot_pos.split(",")[1])
                    robot_pos = (float(robot_pos[0].split("(")[1]), float(robot_pos[1].split(")")[0]))

                    robot_direction = entry["Robot Direction"]
                    robot_direction = [(robot_direction[0].split(",")), (robot_direction[1].split(","))]
                    robot_direction = [(float(robot_direction[0][0].split("(")[1]), float(robot_direction[0][1].split(")")[0])), (float(robot_direction[1][0].split("(")[1]), float(robot_direction[1][1].split(")")[0]))]
                    robot = Robot(robot_pos, robot_direction)
                except:
                    robot = Robot(None, None)

                # Correction Data
                try:
                    correction_direction = entry["Correction Direction"]
                    correction_line = entry["Correction Line"]
                    correction_line = [(correction_line[0].split(",")), (correction_line[1].split(","))]
                    correction_line = [(float(correction_line[0][0].split("(")[1]), float(correction_line[0][1].split(")")[0])), (float(correction_line[1][0].split("(")[1]), float(correction_line[1][1].split(")")[0]))]
                    correction_error = float(entry["Forward Error"])
                    correction_chooser = int(entry["Correction Chooser"])
                    correction = Correction(correction_line, correction_direction, correction_error, correction_chooser)
                except:
                    correction = Correction(None, None, None, None)
                mission_record = MissionRecord(iteration, mission, correction, robot)
                self.records.append(mission_record)
                self.total_records += 1

    def is_in(self, element: "tuple[float, float]", records: "list[MissionRecord]") -> bool:
        for r in records:
            if(r.robot.robot_pos == element):
                return True
        return False

    def clean_duplicated(self) -> None:
        clean_records: "list[MissionRecord]" = []
        for r in self.records:
            if(not self.is_in(r.robot.robot_pos, clean_records)):
                clean_records.append(r)
        self.records = clean_records
        
    def plot_read_data(self) -> None:
        if(len(sys.argv) == 2):
            step = int(sys.argv[1])
        else:
            step = 1
        index = 0
        for r in range(0, len(self.records), step):
            dir_c_p1, dir_c_p2 = self.records[r].correction.line
            dir_c_p1_y, dir_c_p1_x = dir_c_p1
            dir_c_p2_y, dir_c_p2_x = dir_c_p2

            plt.plot([dir_c_p1_x, dir_c_p2_x], [dir_c_p1_y, dir_c_p2_y], marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Linha de correção - iteração: {}/{} - erro: {:.8f}'.format(index, len(self.records), self.records[r].correction.error))

            dir_m_p1, dir_m_p2 = self.records[r].mission.line
            dir_m_p1_y, dir_m_p1_x = dir_m_p1
            dir_m_p2_y, dir_m_p2_x = dir_m_p2

            plt.plot([dir_m_p1_x, dir_m_p2_x], [dir_m_p1_y, dir_m_p2_y], marker='o', markerfacecolor='purple', markersize=6, color='purple', label='Linha da missão - chooser {}'.format(self.records[r].correction.chooser))

            dir_r_p1, dir_r_p2 = self.records[r].robot.direction_line
            dir_r_p1_y, dir_r_p1_x = dir_r_p1
            dir_r_p2_y, dir_r_p2_x = dir_r_p2

            y_a_y_o = dir_r_p2_y - dir_c_p2_y

            plt.plot([dir_r_p1_x, dir_r_p2_x], [dir_r_p1_y, dir_r_p2_y], marker='o', markerfacecolor='orange', markersize=6, color='orange', label='Linha de direção do robô - Correção: {} - Y_A: {:8.7f} - Y_O: {:8.7f}'.format(self.records[r].correction.direction, dir_r_p2_y, dir_c_p2_y))

            plt.annotate("",
                    xy=( dir_r_p2_x, dir_r_p2_y),
                    xytext=(dir_r_p1_x, dir_r_p1_y),
                    va="center",
                    ha="right",
                    arrowprops={"arrowstyle": "-|>", "lw": 1})

            y_r, x_r = self.records[r].robot.robot_pos
            plt.plot(x_r, y_r, marker='o', markerfacecolor='black', markersize=6, color='black', label='Posição atual do robô {:8.7f}, {:8.7f}'.format(y_r, x_r))

            plt.legend()
            plt.show()
            index += 1

    def print_record_data(self) -> None:
        for r in self.records:
            print("Iteration: ", r.iteration)
            print("Mission Name: ", r.mission.name)
            print("Mission Line: ", r.mission.line)
            print("Mission Points: ", r.mission.points, "\n")

            print("Robot Position: ", r.robot.robot_pos)
            print("Robot Direction Line: ", r.robot.direction_line, "\n")

            print("Correction Direction: ", r.correction.direction)
            print("Correction Line: ", r.correction.line)
            print("_" * 150)

    def get_robot_positions(self) -> "list[float, float]":
        robot_positions: "list[float, float]" = []
        for r in range(0, len(self.records)):
            robot_positions.append(self.records[r].robot.robot_pos)

        return robot_positions

    def publish_fake_gps_data(self) -> None:
        gps_points = self.get_robot_positions()
        for g in gps_points:
            c = Coords()
            c.latitude = g[0]
            c.longitude = g[1]
            pub.publish(c)
            time.sleep(0.1)
        pub_stop.publish("stop")

    def size(self) -> int:
        return len(self.records)
        
m = MissionDataAnalyzer("/home/caioslpp/quarta_missao_simulada_com_pontos_reais.json")
m.clean_duplicated()
#m.publish_fake_gps_data()
#m.get_robot_positions()
m.plot_read_data()