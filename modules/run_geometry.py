from geometry import Line, PathCalculator, Point

l = Line()
l.add(Point(0, 0))
l.add(Point(-1, 1))

print(l.angular_coefficient())

l2 = Line()
l2.add(Point(0, 0))
l2.add(Point(1, -1))

print(l2.angular_coefficient())
