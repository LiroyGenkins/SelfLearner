import numpy as np

import constants as const


def point_point_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def point_line_distance(point, line_point1, line_point2):
    x0, y0 = point
    x1, y1 = line_point1
    x2, y2 = line_point2
    try:
        dist = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / \
               np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
        return dist
    except ZeroDivisionError:
        raise ZeroDivisionError('Точки, принадлежащие прямой, должны различаться!')


def point_on_segment(point, line_point1, line_point2):
    if point_line_distance(point, line_point1, line_point2) < const.POSITION_EPSILON:
        return True
    return False


def segments_intersect(sector1, sector2):
    def direction(p1, p2, p3):
        return np.cross(np.subtract(p3, p1)), np.subtract(p2, p1)

    def on_segment(p1, p2, p):
        x1, y1 = p1
        x2, y2 = p2
        x, y = p
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

    p1, p2 = sector1
    p3, p4 = sector2

    d1 = direction(p3, p4, p1)
    d2 = direction(p3, p4, p2)
    d3 = direction(p1, p2, p3)
    d4 = direction(p1, p2, p4)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
            ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    elif d1 == 0 and on_segment(p3, p4, p1):
        return True
    elif d2 == 0 and on_segment(p3, p4, p2):
        return True
    elif d3 == 0 and on_segment(p1, p2, p3):
        return True
    elif d4 == 0 and on_segment(p1, p2, p4):
        return True
    else:
        return False

# def _ramer_douglas_peucker(self, line, eps):
#     indices_stack = [(0, len(line))]
#     global_start_index = 0
#
#     new_line = []
#
#     while len(indices_stack) > 0:
#         start_index, last_index = indices_stack.pop()
#
#         dmax = 0.
#         index = start_index
#
#         for i in range(index + 1, last_index):
#             if new_line[i]
