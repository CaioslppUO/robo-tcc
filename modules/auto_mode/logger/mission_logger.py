"""
@package mission_logger.py
Register the last executed mission in a log file.
"""

from pathlib import Path
import sys, json

class MissionLogger:
    def __init__(self) -> None:
        aux = sys.argv[0].split("/")[0:-1]
        pth = ""
        for i in aux:
            pth += i + "/"
        self.mission_log_file = Path(pth).absolute().joinpath("logger/mission_log.json")

        with open(self.mission_log_file, "w") as file: # Clean the log files
            file.write("[\n]")
        
        # Robot Position
        self.current_robot_latitude = 0.0
        self.current_robot_longitude = 0.0

        # Robot Direction Line
        self.current_robot_direction_p1_latitude = 0.0
        self.current_robot_direction_p1_longitude = 0.0

        self.current_robot_direction_p2_latitude = 0.0
        self.current_robot_direction_p2_longitude = 0.0

        # Correction Line
        self.correction_line_p1_latitude = 0.0
        self.correction_line_p1_longitude = 0.0

        self.correction_line_p2_latitude = 0.0
        self.correction_line_p2_longitude = 0.0

        # Mission Line
        self.mission_line_p1_latitude = 0.0
        self.mission_line_p1_longitude = 0.0

        self.mission_line_p2_latitude = 0.0
        self.mission_line_p2_longitude = 0.0

        # Mission Points (Calculated)
        self.mission_points: "list[float, float]" = []

        # Correction Direction
        self.correction_direction = ""

    def do_log(self) -> None:
        """
        Append log to the logs file.
        """
        with open(self.mission_log_file) as file:
            list_obj = json.load(file)

        list_obj.append({
            "Robot Location": "({:10.7f}, {:10.7f})".format(self.current_robot_latitude, self.current_robot_longitude),
            "Robot Direction": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.current_robot_direction_p1_latitude, self.current_robot_direction_p1_longitude, self.current_robot_direction_p2_latitude, self.current_robot_direction_p2_longitude),
            "Correction Line": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.correction_line_p1_latitude, self.correction_line_p1_longitude, self.correction_line_p2_latitude, self.correction_line_p2_longitude),
            "Correction Direction": "{}".format(self.correction_direction),
            "Mission Line": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.mission_line_p1_latitude, self.mission_line_p1_longitude, self.mission_line_p2_latitude, self.mission_line_p2_longitude),
            "Mission Points": "{}".format(self.mission_points)
        })

        with open(self.mission_log_file, "w") as file:
            json.dump(list_obj, file, indent=4, separators=(',', ': '))
    