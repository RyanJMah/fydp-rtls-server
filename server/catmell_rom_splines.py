import numpy as np
from typing import List, Tuple

# taken mostly from wikipedia (lol): https://en.wikipedia.org/wiki/Centripetal_Catmull–Rom_spline

QUADRUPLE_SIZE: int = 4

def num_segments(point_chain: tuple) -> int:
    # There is 1 segment per 4 points, so we must subtract 3 from the number of points  
    return len(point_chain) - (QUADRUPLE_SIZE - 1)


def flatten(list_of_lists) -> list:
    # E.g. mapping [[1, 2], [3], [4, 5]] to  [1, 2, 3, 4, 5] 
    return [elem for lst in list_of_lists for elem in lst]


def calculate_num_points(points: List[Tuple[float, float]], factor: float) -> int:
    """
    Calculate the number of interpolated points based on the length of points and the distance between the start and end point.
    :param points: Base points from which the quadruples for the algorithm are taken
    :param factor: A constant factor to adjust the density of interpolated points
    :return: The number of interpolated points
    """
    start_point = np.array(points[0])
    end_point = np.array(points[-1])
    distance = np.linalg.norm(end_point - start_point)
    num_points = int(distance / len(points) * factor)
    return num_points

# Calculate t0 to t4. Then only calculate points between P1 and P2.
# Reshape linspace so that we can multiply by the points P0 to P3
# and get a point for each value of t.
def tj(ti: float, pi: tuple, pj: tuple, alpha: float = 0.5) -> float:
    xi, yi = pi
    xj, yj = pj
    dx, dy = xj - xi, yj - yi
    # l = (dx ** 2 + dy ** 2) ** 0.5
    l = np.hypot(dx, dy)
    return ti + l ** alpha


def catmull_rom_spline(
    P0: tuple,
    P1: tuple,
    P2: tuple,
    P3: tuple,
    num_points: int,
    alpha: float = 0.5,
):
    """
    Compute the points in the spline segment
    :param P0, P1, P2, and P3: The (x,y) point pairs that define the Catmull-Rom spline
    :param num_points: The number of points to include in the resulting curve segment
    :param alpha: 0.5 for the centripetal spline, 0.0 for the uniform spline, 1.0 for the chordal spline.
    :return: The points
    """

    t0: float = 0.0
    t1: float = tj(t0, P0, P1, alpha=alpha)
    t2: float = tj(t1, P1, P2, alpha=alpha)
    t3: float = tj(t2, P2, P3, alpha=alpha)
    t = np.linspace(t1, t2, num_points).reshape(num_points, 1)

    A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1
    A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2
    A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3
    B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
    B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
    points = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2
    return points


def catmull_rom_chain(points: List[Tuple[float, float]], points_per_joint: float = 10) -> list:
    """
    Calculate Catmull-Rom for a sequence of initial points and return the combined curve.
    :param points: Base points from which the quadruples for the algorithm are taken
    :param num_points: The number of points to include in each curve segment
    :return: The chain of all points (points of all segments)
    """

    # Catmull-ROM splines don't connect the endpoints of the curve, so
    # create "ghost" points on the ends to force a connection
    #
    # Mirror the second point around the first point.
    #
    # Similarly, mirror the second-to-last point around the last point.

    # First point, minus the x distance between the second and first point
    first_ghost_x = points[0][0] - (points[1][0] - points[0][0])

    # First point, minus the y distance between the second and first point
    first_ghost_y = points[0][1] - (points[1][1] - points[0][1])

    # Last point, plus the x distance between the second-to-last and last point
    last_ghost_x = points[-1][0] + (points[-1][0] - points[-2][0])

    # Last point, plus the y distance between the second-to-last and last point
    last_ghost_y = points[-1][1] + (points[-1][1] - points[-2][1])

    # Create a new array of points with the ghost points added
    new_points = [ (first_ghost_x, first_ghost_y),  *points , (last_ghost_x, last_ghost_y)]

    point_quadruples = (  # Prepare function inputs
        (new_points[idx_segment_start + d] for d in range(QUADRUPLE_SIZE))
        for idx_segment_start in range(num_segments(new_points))
    )

    # num_points = calculate_num_points(points, points_per_cm)
    # print(f"num_points: {num_points}")

    all_splines = (catmull_rom_spline(*pq, points_per_joint) for pq in point_quadruples)
    return flatten(all_splines)