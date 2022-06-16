"""
@package process_lidar.py
Process lidar data and return the values from its processing.
"""

class ProcessLidar():

    def select_points(self, array: "list[int]", range: int, central_point: int) -> "list[int]":
        """
        Select the points in the front of the robot based on lidar data.
        """
        i = 1
        vet: "list[int]" = []
        vet.append(array[central_point])
        while (i < range):
            vet.append(array[central_point + i])
            vet.append(array[central_point - i])
            i = i + 1
        return vet

    def get_closest_distance(self, array: "list[int]") -> int:
        """
        Return the smallest distance in the array.
        """
        return min(array)