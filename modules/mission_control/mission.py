#!/usr/bin/env python
"""
@package mission.py
Mission class.
"""
import json

class _Location:
    """
    Represent a location with an action to be executed.
    """

    def __init__(self, latitude: float, longitude: float, action: str) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.action = action


class _Mission:
    """
    Represent an mission to be executed, with all its locations.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.locations: list[_Location] = []

    def add_location(self, latitude: float, longitude: float, order: int, action: str) -> None:
        self.locations.insert(order, _Location(latitude, longitude, action))

    def get_locations(self) -> "list[_Location]":
        return self.locations

    def get_location(self, index: int) -> _Location:
        return self.locations[index]


class Missions:
    """
    Represents all missions to be executed.
    """

    def __init__(self) -> None:
        self.missions: list[_Mission] = []

    def add_mission(self, name: str, order: int) -> None:
        self.missions.insert(order, _Mission(name))

    def get_missions(self) -> "list[_Mission]":
        return self.missions

    def get_mission(self, index: int) -> _Mission:
        return self.missions[index]

    def show(self) -> None:
        i = 0
        for mission in self.missions:
            print("name: {}".format(mission.name))
            print("order: {}".format(i), end="\nLocations: \n    [\n")
            j = 0
            for location in self.missions[i].get_locations():
                print("       {")
                print("         latitude: {}".format(location.latitude))
                print("         longitude: {}".format(location.longitude))
                print("         order: {}".format(j))
                print("\n       },")
                j += 1
            print("    ]")
            i += 1
            print("\n")

    def load_mission_file(self) -> None:
        with open('mission.json') as f:
            data = json.load(f)
            for entry in data:
                mission_name = entry["name"]
                mission_order = entry["order"]
                locations = entry["locations"]
                self.add_mission(mission_name, mission_order)
                for location in locations:
                    latitude = location["latitude"]
                    longitude = location["longitude"]
                    location_order = location["order"]
                    action = location["action"]
                    self.get_mission(mission_order).add_location(latitude, longitude, location_order, action)