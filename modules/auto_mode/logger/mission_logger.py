"""
@package mission_logger.py
Register the last executed mission in a log file.
"""

from pathlib import Path
import json, os.path
import datetime

class MissionLogger:
    def __init__(self) -> None:
        self.mission_log_file = Path.home().joinpath("Agrobot/mission_logs")
        self.date_no_spaces = str(datetime.datetime.now()).replace(" ", "_")
        
        # Mission Name
        self.mission_name = ""

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

        # Iteration
        self.iteration = 0

        # Correction Direction
        self.correction_direction = ""

        # Forward error
        self.forward_error: float = 0
        self.correction_chooser: int = -2

    def update_robot_location(self, latitude: float, longitude: float) -> None:
        self.current_robot_latitude = latitude
        self.current_robot_longitude = longitude

    def update_robot_direction(self, p1_lat: float, p1_lon: float, p2_lat: float, p2_lon: float) -> None:
        self.current_robot_direction_p1_latitude = p1_lat
        self.current_robot_direction_p1_longitude = p1_lon
        self.current_robot_direction_p2_latitude = p2_lat
        self.current_robot_direction_p2_longitude = p2_lon

    def update_correction_line(self, p1_lat: float, p1_lon: float, p2_lat: float, p2_lon: float) -> None:
        self.correction_line_p1_latitude = p1_lat
        self.correction_line_p1_longitude = p1_lon
        self.correction_line_p2_latitude = p2_lat
        self.correction_line_p2_longitude = p2_lon

    def update_mission_line(self, p1_lat: float, p1_lon: float, p2_lat: float, p2_lon: float) -> None:
        self.mission_line_p1_latitude = p1_lat
        self.mission_line_p1_longitude = p1_lon
        self.mission_line_p2_latitude = p2_lat
        self.mission_line_p2_longitude = p2_lon

    def update_mission_points(self, points: list) -> None:
        self.mission_points = []
        for p in points:
            self.mission_points.append((p.latitude, p.longitude))

    def update_correction_direction(self, correction_direction: str) -> None:
        self.correction_direction = correction_direction

    def update_mission_name(self, name: str) -> None:
        self.mission_name = name

    def update_iteration(self, iteration: int) -> None:
        self.iteration = iteration

    def update_forward_error(self, error: float) -> None:
        self.forward_error = error

    def update_correction_chooser(self, correction_chooser: int) -> None:
        self.correction_chooser = correction_chooser

    def get_log_file(self) -> str:
        """
        Return the log file location.
        """
        no_space_mission_name = self.mission_name.replace(" ", "_")
        mission_log_file = self.mission_log_file.joinpath("{}-{}_log_file.json".format(no_space_mission_name, self.date_no_spaces))
        return mission_log_file
    
    def do_log(self) -> str:
        """
        Append log to the logs file. Return the mission file name.
        """
        no_space_mission_name = self.mission_name.replace(" ", "_")
        mission_log_file = self.mission_log_file.joinpath("{}-{}_log_file.json".format(no_space_mission_name, self.date_no_spaces))

        # Create file if not exists.
        if(not os.path.isfile(mission_log_file)):
            with open(mission_log_file, "w") as file:
                file.write("[\n]")

        with open(mission_log_file) as file:
            list_obj = json.load(file)

        list_obj.append({
            "Iteration": "{}".format(self.iteration),
            "Mission Name": "{}".format(self.mission_name),
            "Robot Location": "({:10.7f}, {:10.7f})".format(self.current_robot_latitude, self.current_robot_longitude),
            "Robot Direction": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.current_robot_direction_p1_latitude, self.current_robot_direction_p1_longitude, self.current_robot_direction_p2_latitude, self.current_robot_direction_p2_longitude),
            "Correction Line": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.correction_line_p1_latitude, self.correction_line_p1_longitude, self.correction_line_p2_latitude, self.correction_line_p2_longitude),
            "Correction Direction": "{}".format(self.correction_direction),
            "Forward Error":  "{}".format(self.forward_error),
            "Correction Chooser": "{}".format(self.correction_chooser),
            "Mission Line": "({:10.7f}, {:10.7f}) -> ({:10.7f}, {:10.7f})".format(self.mission_line_p1_latitude, self.mission_line_p1_longitude, self.mission_line_p2_latitude, self.mission_line_p2_longitude),
            "Mission Points": "{}".format(self.mission_points)
        })

        with open(mission_log_file, "w") as file:
            json.dump(list_obj, file, indent=4, separators=(',', ': '))
    
        return mission_log_file