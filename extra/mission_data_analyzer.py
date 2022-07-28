"""
@package mission_data_analyzer.
Able to read mission_log.json files and plot what happened to the robot.
"""

import json
import matplotlib.pyplot as plt

class Mission:
    def __init__(self, mission_name: str, mission_line: "tuple[tuple[float, float], tuple[float, float]]", mission_points: "list[tuple[float, float]]") -> None:
        self.name = mission_name
        self.line = mission_line
        self.points = mission_points
        
class Correction:
    def __init__(self, correction_line: "tuple[tuple[float, float], tuple[float, float]]", correction_direction: str) -> None:
        self.line = correction_line
        self.direction = correction_direction

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
        self.__organize_file()
        self.__load_data()

    def __organize_file(self) -> None:
        data = None
        with open(self.logs_file, "r") as f:
            data = json.load(f)
            index = 0
            for entry in data:
                # Index
                entry["Iteration:"] = index
                index += 1

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
            for entry in data:
                # Mission Data
                mission_name = entry["Mission Name:"]
                mission_line = entry["Mission Line"]
                mission_line = [(mission_line[0].split(",")), (mission_line[1].split(","))]
                mission_line = [(float(mission_line[0][0].split("(")[1]), float(mission_line[0][1].split(")")[0])), (float(mission_line[1][0].split("(")[1]), float(mission_line[1][1].split(")")[0]))]
                mission_points = entry["Mission Points"]
                mission = Mission(mission_name, mission_line, mission_points)

                # Iteration
                iteration = entry["Iteration:"]

                # Robot Data
                robot_pos = entry["Robot Location"]
                robot_pos = (robot_pos.split(",")[0], robot_pos.split(",")[1])
                robot_pos = (float(robot_pos[0].split("(")[1]), float(robot_pos[1].split(")")[0]))

                robot_direction = entry["Robot Direction"]
                robot_direction = [(robot_direction[0].split(",")), (robot_direction[1].split(","))]
                robot_direction = [(float(robot_direction[0][0].split("(")[1]), float(robot_direction[0][1].split(")")[0])), (float(robot_direction[1][0].split("(")[1]), float(robot_direction[1][1].split(")")[0]))]
                robot = Robot(robot_pos, robot_direction)

                # Correction Data
                correction_direction = entry["Correction Direction"]
                correction_line = entry["Correction Line"]
                correction_line = [(correction_line[0].split(",")), (correction_line[1].split(","))]
                correction_line = [(float(correction_line[0][0].split("(")[1]), float(correction_line[0][1].split(")")[0])), (float(correction_line[1][0].split("(")[1]), float(correction_line[1][1].split(")")[0]))]
                correction = Correction(correction_line, correction_direction)

                mission_record = MissionRecord(iteration, mission, correction, robot)
                self.records.append(mission_record)

    def plot_read_data(self) -> None:
        for r in self.records:
            dir_c_p1, dir_c_p2 = r.correction.line
            dir_c_p1_y, dir_c_p1_x = dir_c_p1
            dir_c_p2_y, dir_c_p2_x = dir_c_p2

            plt.plot([dir_c_p1_x, dir_c_p2_x], [dir_c_p1_y, dir_c_p2_y], marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Linha de correção - iteração: {}/{}'.format(r.iteration, len(self.records)-1))

            dir_m_p1, dir_m_p2 = r.mission.line
            dir_m_p1_y, dir_m_p1_x = dir_m_p1
            dir_m_p2_y, dir_m_p2_x = dir_m_p2

            plt.plot([dir_m_p1_x, dir_m_p2_x], [dir_m_p1_y, dir_m_p2_y], marker='o', markerfacecolor='purple', markersize=6, color='purple', label='Linha da missão')

            dir_r_p1, dir_r_p2 = r.robot.direction_line
            dir_r_p1_y, dir_r_p1_x = dir_r_p1
            dir_r_p2_y, dir_r_p2_x = dir_r_p2

            plt.plot([dir_r_p1_x, dir_r_p2_x], [dir_r_p1_y, dir_r_p2_y], marker='o', markerfacecolor='orange', markersize=6, color='orange', label='Linha de direção do robô - Correção: {}'.format(r.correction.direction))

            y_r, x_r = r.robot.robot_pos
            plt.plot(x_r, y_r, marker='o', markerfacecolor='black', markersize=6, color='black', label='Posição atual do robô {:8.7f}, {:8.7f}'.format(y_r, x_r))

            plt.legend()
            plt.show()

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
        
m = MissionDataAnalyzer("/home/caioslpp/Downloads/erro2virardireita.json")
#m.print_record_data()
m.plot_read_data()