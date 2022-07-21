"""
@package monitor_auto_mode.py
Use the terminal to monitor robot data while in auto_mode.
"""

#!/usr/bin/env python

import traceback
from threading import Thread
from rich.live import Live
from rich.table import Table
import time

DEFAULT_CONTROL_VALUE: float = -9.9
DEFAULT_COORD_VALUE: float = -99.9999999999
DEFAULT_COEFFICIENT_VALUE: float = -99.9999999999
DEFAULT_DISTANCE: float = -99.9999999999
DEFAULT_NUMBER_OF_POINTS: float = -999
DEFAULT_ID: int = -1

class Control:
    def __init__(self) -> None:
        self.speed: float = DEFAULT_CONTROL_VALUE
        self.steer: float = DEFAULT_CONTROL_VALUE
        self.limit: float = DEFAULT_CONTROL_VALUE

class Coordinate:
    def __init__(self) -> None:
         self.latitude: float = DEFAULT_COORD_VALUE
         self.longitude: float = DEFAULT_COORD_VALUE
         self.id: int = DEFAULT_ID

class Coefficients:
    def __init__(self) -> None:
         self.angular_coefficient: float = DEFAULT_COEFFICIENT_VALUE
         self.linear_coefficient: float = DEFAULT_COEFFICIENT_VALUE

class Distance:
    def __init__(self) -> None:
         self.distance = DEFAULT_DISTANCE

class NumberOfPoints:
    def __init__(self) -> None:
         self.number_of_points = DEFAULT_NUMBER_OF_POINTS

class Monitor:
    def __init__(self) -> None:
        self.control = Control()

        self.mission_start = Coordinate()
        self.mission_end = Coordinate()

        self.start_point = Coordinate()
        self.end_point = Coordinate()

        self.coefficients = Coefficients()
        self.distance = Distance()
        self.number_of_points =NumberOfPoints()

        self.robot_point = Coordinate()
        self.closest_point = Coordinate()
        self.correction_point = Coordinate()

    def generate_table(self) -> Table:
        table = Table()
        table.add_column("module")
        table.add_column("speed")
        table.add_column("steer")
        table.add_column("limit")
        table.add_column("latitude")
        table.add_column("longitude")
        table.add_column("angular coeff.")
        table.add_column("linear coeff.")
        table.add_column("total distance")
        table.add_column("points")
        table.add_column("point id")

        table.add_row("Control",
            "{0:1.1f}".format(self.control.speed),
            "{0:1.1f}".format(self.control.steer),
            "{0:1.1f}".format(self.control.limit),
            "",
            "",
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("Mission Start",
            "",
            "",
            "",
            "{:10.10f}".format(self.mission_start.latitude),
            "{:10.10f}".format(self.mission_start.longitude),
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("Mission End",
            "",
            "",
            "",
            "{:10.10f}".format(self.mission_end.latitude),
            "{:10.10f}".format(self.mission_end.longitude),
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("Start Point",
            "",
            "",
            "",
            "{:10.10f}".format(self.start_point.latitude),
            "{:10.10f}".format(self.start_point.longitude),
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("End Point",
            "",
            "",
            "",
            "{:10.10f}".format(self.end_point.latitude),
            "{:10.10f}".format(self.end_point.longitude),
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("Coefficients",
            "",
            "",
            "",
            "",
            "",
            "{:10.10f}".format(self.coefficients.angular_coefficient),
            "{:10.10f}".format(self.coefficients.linear_coefficient),
            "",
            "",
            ""
        )

        table.add_row("Total Distance",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "{:10.10f}".format(self.distance.distance),
            "",
            ""
        )

        table.add_row("Number of Points",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "{:3}".format(self.number_of_points.number_of_points),
            ""
        )

        table.add_row("Robot Location",
            "",
            "",
            "",
            "{:10.10f}".format(self.robot_point.latitude),
            "{:10.10f}".format(self.robot_point.longitude),
            "",
            "",
            "",
            "",
            ""
        )

        table.add_row("Closest Point",
            "",
            "",
            "",
            "{:10.10f}".format(self.closest_point.latitude),
            "{:10.10f}".format(self.closest_point.longitude),
            "",
            "",
            "",
            "",
            "{}".format(self.closest_point.id)
        )

        table.add_row("Correction Point",
            "",
            "",
            "",
            "{:10.10f}".format(self.correction_point.latitude),
            "{:10.10f}".format(self.correction_point.longitude),
            "",
            "",
            "",
            "",
            "{}".format(self.correction_point.id)
        )
        
        return table

    def update_table(self, live: Live) -> None:
        """
        Update the table control.
        """
        live.update(self.generate_table())
    
    def run(self) -> None:
        try:
            with Live(self.generate_table(), refresh_per_second=4) as live:
                    while True:
                        try:
                            self.update_table(live)
                        except KeyboardInterrupt:
                            exit(0)
        except KeyboardInterrupt:
            exit(0)