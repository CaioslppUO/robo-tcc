"""
@package mission_data_analyzer.
Able to read mission_log.json files and analyze what happened.
"""

from cProfile import label
import json, sys, traceback
from mission_data import Correction, Mission, MissionRecord, Robot

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
                    mission_name = entry["Mission Name:"]
                    mission_line = entry["Mission Line"]
                    mission_line = [(mission_line[0].split(",")), (mission_line[1].split(","))]
                    mission_line = [(float(mission_line[0][0].split("(")[1]), float(mission_line[0][1].split(")")[0])), (float(mission_line[1][0].split("(")[1]), float(mission_line[1][1].split(")")[0]))]
                    mission_points = entry["Mission Points"]
                    mission = Mission(mission_name, mission_line, mission_points)
                except:
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
                    iteration = entry["Iteration:"]
                except:
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
        print(self.records[-1].correction.direction)
        for r in self.records:
            if(not self.__is_in(r.robot.robot_pos, clean_records) or r.correction.direction == "Reached Point"):
                clean_records.append(r)

        self.records = clean_records

        # Update total elements
        self.total_records = len(self.records)
        
    def plot_read_data(self) -> None:
        """
        Plot the read data from the log file.
        """
        try:
            import matplotlib.pyplot as plt
            import seaborn as sns  
            sns.set_style("whitegrid")
        except:
            print(traceback.format_exc())
        try:
            if(len(sys.argv) == 3):
                step = int(sys.argv[2])
            else:
                step = 1
            index = 0
            for r in range(0, len(self.records), step):
                # Mission Line
                dir_m_p1, dir_m_p2 = self.records[r].mission.line
                dir_m_p1_y, dir_m_p1_x = dir_m_p1
                dir_m_p2_y, dir_m_p2_x = dir_m_p2

                plt.plot([dir_m_p1_x, dir_m_p2_x], [dir_m_p1_y, dir_m_p2_y], marker='o', markerfacecolor='red', markersize=6, color='red', label='Vetor da Missão')

                plt.annotate("",
                        xy=(dir_m_p2_x, dir_m_p2_y),
                        xytext=(dir_m_p1_x, dir_m_p1_y),
                        va="center",
                        ha="right",
                        arrowprops={"arrowstyle": "-|>", "lw": 1, "color": "red"}, 
                        color="red")

                # Correction Line
                dir_c_p1, dir_c_p2 = self.records[r].correction.line
                dir_c_p1_y, dir_c_p1_x = dir_c_p1
                dir_c_p2_y, dir_c_p2_x = dir_c_p2

                plt.plot([dir_c_p1_x, dir_c_p2_x], [dir_c_p1_y, dir_c_p2_y], marker='o', markerfacecolor='blue', markersize=6, color='blue', label='Vetor do Ponto de Correção - {} - Iteração {}/{}'.format(self.records[r].correction.chooser, index,self.total_records-1))

                plt.annotate("",
                        xy=(dir_c_p2_x, dir_c_p2_y),
                        xytext=(dir_c_p1_x, dir_c_p1_y),
                        va="center",
                        ha="right",
                        arrowprops={"arrowstyle": "-|>", "lw": 1, "color": "blue"}, 
                        color="blue",
                        textcoords='data')

                ## Robot Position
                y_r, x_r = self.records[r].robot.robot_pos

                # Robot Direction Line
                dir_r_p1, dir_r_p2 = self.records[r].robot.direction_line
                dir_r_p1_y, dir_r_p1_x = dir_r_p1
                dir_r_p2_y, dir_r_p2_x = dir_r_p2

                if(self.records[r].correction.direction == "clockwise"):
                    correction_direction = "direita"
                elif(self.records[r].correction.direction == "counter_clockwise"):
                    correction_direction = "esquerda"
                elif(self.records[r].correction.direction == "none"):
                    correction_direction = "reto"
                elif(self.records[r].correction.direction == "Reached Point"):
                    correction_direction = "chegou ao destino"
                else:
                    correction_direction = "unknown"

                dif_x = (dir_r_p2_x - dir_r_p1_x) * 4
                dif_y = (dir_r_p2_y - dir_r_p1_y) * 4

                plt.plot([x_r, dir_r_p2_x + dif_x*0.94], [y_r, dir_r_p2_y + dif_y*0.94], marker='o', markerfacecolor='green', markersize=0, color='green', label='Vetor de Direção do Robô - Correção: {}'.format(correction_direction))

                plt.annotate("",
                        xy=(dir_r_p2_x + dif_x, dir_r_p2_y + dif_y),
                        xytext=(x_r, y_r),
                        va="center",
                        ha="right",
                        arrowprops={"arrowstyle": "-|>", "lw": 2, "color": "green"},
                        color="green")

                plt.plot(x_r, y_r, marker='o', markerfacecolor='black', markersize=6, color='black', label='Posição Atual do Robô {:8.7f}, {:8.7f}'.format(y_r, x_r))
                plt.ticklabel_format(useOffset=False)

                plt.title("Simulação de Controle Autônomo com Pontos Reais de GPS", fontsize=18)

                plt.ylabel("Latitude", fontsize=16)
                plt.xlabel("Longitude", fontsize=16)
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

if __name__ == "__main__":
    try:
        file = sys.argv[1]
        m = MissionDataAnalyzer(file)
        m.plot_read_data()
    except:
        print(traceback.format_exc())