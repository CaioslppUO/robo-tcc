"""
@package mission_data_analyzer.
Able to read mission_log.json files and analyze what happened.
"""

import json, sys, traceback, time
from .mission_data import Correction, Mission, MissionRecord, Robot

try:
    import matplotlib.pyplot as plt
except:
    print(traceback.format_exc())

class MissionDataAnalyzer:
    def __init__(self, logs_file_path: str, clean_duplicated: bool = True) -> None:
        self.logs_file = logs_file_path
        self.records: "list[MissionRecord]" = []
        self.total_records = 0
        self.__organize_file()
        self.__load_data()

        if(clean_duplicated):
            self.__clean_duplicated()

    def __organize_file(self) -> None:
        """
        Translate the javascript objects to python lists and tuples in the mission log file.
        """
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
        """
        Load the data from a mission log file to the memory.
        """
        with open(self.logs_file, "r") as f:
            data = json.load(f)
            i = 0
            self.total_records = 0
            success_mission = True
            success_robot = True
            success_correction = True
            success_iteration = True
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
                    if(success_mission):
                        success_mission = False
                        print(traceback.format_exc())
                    mission = Mission(None, None, None)

                # Iteration
                try:
                    iteration = entry["Iteration"]
                except:
                    if(success_iteration):
                        success_iteration = False
                        print(traceback.format_exc())
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
                    if(success_robot):
                        success_robot = False
                        print(traceback.format_exc())
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
                    if(success_correction):
                        success_correction = False
                        print(traceback.format_exc())
                    correction = Correction(None, None, None, None)

                mission_record = MissionRecord(iteration, mission, correction, robot)
                self.records.append(mission_record)
                self.total_records += 1

    def __is_in(self, element: "tuple[float, float]", records: "list[MissionRecord]") -> bool:
        """
        Return if a record (robot latitude, longitude) is inside the records list.
        """
        for r in records:
            if(r.robot.robot_pos == element):
                return True
        return False

    def __clean_duplicated(self) -> None:
        """
        Remove duplicated elements (robot latitude and longitude) in the records.
        """
        clean_records: "list[MissionRecord]" = []
        for r in self.records:
            if(not self.__is_in(r.robot.robot_pos, clean_records)):
                clean_records.append(r)
        self.records = clean_records

        # Update total elements
        self.total_records = len(self.records)
        
    def plot_read_data(self) -> None:
        """
        Plot the read data from the log file.
        """
        try:
            if(len(sys.argv) == 2):
                step = int(sys.argv[1])
            else:
                step = 1
            index = 0
            for r in range(0, len(self.records), step):
                # Correction Line
                dir_c_p1, dir_c_p2 = self.records[r].correction.line
                dir_c_p1_y, dir_c_p1_x = dir_c_p1
                dir_c_p2_y, dir_c_p2_x = dir_c_p2

                plt.plot([dir_c_p1_x, dir_c_p2_x], [dir_c_p1_y, dir_c_p2_y], marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Linha de correção - iteração: {}/{}'.format(index, self.total_records, self.records[r].correction.error))

                plt.annotate("",
                        xy=( dir_c_p2_x, dir_c_p2_y),
                        xytext=(dir_c_p1_x, dir_c_p1_y),
                        va="center",
                        ha="right",
                        arrowprops={"arrowstyle": "-|>", "lw": 1, "color": "blue"}, 
                        color="blue")

                # Mission Line
                dir_m_p1, dir_m_p2 = self.records[r].mission.line
                dir_m_p1_y, dir_m_p1_x = dir_m_p1
                dir_m_p2_y, dir_m_p2_x = dir_m_p2

                plt.plot([dir_m_p1_x, dir_m_p2_x], [dir_m_p1_y, dir_m_p2_y], marker='o', markerfacecolor='red', markersize=6, color='red', label='Linha da missão')

                dir_r_p1, dir_r_p2 = self.records[r].robot.direction_line
                dir_r_p1_y, dir_r_p1_x = dir_r_p1
                dir_r_p2_y, dir_r_p2_x = dir_r_p2

                # Robot Direction Line
                if(self.records[r].correction.direction == "clockwise"):
                    correction_direction = "horário"
                elif(self.records[r].correction.direction == "counter_clockwise"):
                    correction_direction = "anti horário"
                elif(self.records[r].correction.direction == "none"):
                    correction_direction = "reto"

                plt.plot([dir_r_p1_x, dir_r_p2_x], [dir_r_p1_y, dir_r_p2_y], marker='o', markerfacecolor='green', markersize=6, color='green', label='Linha de direção do robô - Correção: {}'.format(correction_direction))

                plt.annotate("",
                        xy=( dir_r_p2_x, dir_r_p2_y),
                        xytext=(dir_r_p1_x, dir_r_p1_y),
                        va="center",
                        ha="right",
                        arrowprops={"arrowstyle": "-|>", "lw": 1, "color": "green"},
                        color="green")

                # Robot Position
                y_r, x_r = self.records[r].robot.robot_pos
                plt.plot(x_r, y_r, marker='o', markerfacecolor='black', markersize=6, color='black', label='Posição atual do robô {:8.7f}, {:8.7f}'.format(y_r, x_r))

                plt.legend()
                plt.show()
                index += 1
        except:
            print(traceback.format_exc())

    def print_record_data(self) -> None:
        """
        Print in the terminal all loaded data from the mission log file.
        """
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

    def get_gps_robot_positions(self) -> "list[float, float]":
        """
        Return a list with all robot gps locations that were generated during the execution of a mission.
        These locations are extracted from the log file generated in each mission execution.
        """
        robot_positions: "list[float, float]" = []
        for r in range(0, len(self.records)):
            robot_positions.append(self.records[r].robot.robot_pos)

        return robot_positions

    def size(self) -> int:
        """
        Return the amount of read records.
        """
        return len(self.records)