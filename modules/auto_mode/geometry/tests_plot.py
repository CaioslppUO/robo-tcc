from pathlib import Path
from graph import plot
from path_calculator import PathCalculator, Point, Points

def test_1() -> None:
    decimals = 10
    # Up left
    #start = Point(-25.435348, -54.596970, decimals)
    #end = Point(-25.435324, -54.5969705, decimals)

    # Up right
    #start = Point(-25.435348, -54.596970, decimals)
    #end = Point(-25.435324, -54.596960, decimals)

    # Down left
    #start = Point(-25.435348, -54.596970, decimals)
    #end = Point(-25.435355, -54.5969705, decimals)

    # Down right
    start = Point(-25.435348, -54.596970, decimals)
    end = Point(-25.435355, -54.596960, decimals)
    path = PathCalculator(decimals, start, end, 10)

    points = path.get_points_between()
    plot(points)

test_1()