import matplotlib.pyplot as plt
import random
import math
import time
import sys

def generate_points(n = 20):
    """Takes in a number of points and generates a random point set of num_points size"""
    return [(random.gauss(0, 10), random.gauss(0, 10)) for _ in range(n)]

def sort_points_counter_clockwise(points):
    # Compute the centroid of the points
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)

    # Sort the points based on their angle with respect to the centroid
    points_sorted = sorted(points, key=lambda p: (math.atan2(p[1] - cy, p[0] - cx) + 2 * math.pi) % (2 * math.pi))

    return points_sorted

def find_angle(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    angle = math.atan2(dy, dx)
    return math.degrees(angle)

def sort_by_angles(p, points, values = False):
    """Finds the relative angles of a point set for the gift-wrapping algorithm"""
    angles = {}
    for point in points:
        angle = find_angle(point, p)
        angles[angle] = point
     
    return [angles[key] if values == False else (angles[key], key) for key in sorted(angles.keys())]        
    
def poly_clip(polygon, point):
    """Clips the points on a polygon off by finding the tangents"""
    new_p = polygon + [point]
    counter_clockwise = sort_points_counter_clockwise(new_p)
    index = counter_clockwise.index(point)
    counter_clockwise = counter_clockwise[index:] + counter_clockwise[:index]

    sorted_angles = sort_by_angles(point, polygon, values = True)
    sorted_angles.sort(key = lambda x: x[1])

    neg = [item for item in sorted_angles if item[1] < 0]
    pos = [item for item in sorted_angles if item[1] >= 0]

    if neg and pos:
        max_min = [neg[-1][0], pos[0][0]]
    elif neg:
        max_min = [neg[0][0], neg[-1][0]]
    elif pos:
        max_min = [pos[0][0], pos[-1][0]]

    left = counter_clockwise.index(max_min[0])
    right = counter_clockwise.index(max_min[1])

    if left > right:
        counter_clockwise = counter_clockwise[slice(left, right - 1, -1)]
    else:
        counter_clockwise = counter_clockwise[left:right + 1]

    if point not in counter_clockwise:
        counter_clockwise.append(point)
    
    return counter_clockwise

def incremental(n, viz = True):
    # Sort the points lexicographically
    points = generate_points(n)
    points = sorted(points)
    hull = points[:3]

    if viz:
        plot_init(points, hull)

    # Iterate over the remaining points
    for point in points[3:]:
        hull = poly_clip(hull, point)

        if viz:
            plot_update(points, hull)

    if viz:
        plot_final(points, hull)

    return hull


def find_max_angle(p, points, inv = False):
    """Finds the relative angles of a point set for the gift-wrapping algorithm"""
    angles = {}
    for point in points:
        angle = find_angle(point, p)

        if not inv:
            if angle >= 0:
                angles[angle] = point
        else:
            if angle < 0:
                angles[abs(angle)] = point

    if not angles:
        return None
    elif inv:
        max_angle = max(angles.keys())
    elif len(angles):
        max_angle = min(angles.keys())

    return (max_angle, angles[max_angle])

def gift_wrapping(n, viz =  True):
    """finds the convex hull of a set using gift-wrapping"""
    points = generate_points(n)
    init = points.copy()
    points = sorted(points, key=lambda x: x[1])
    final = points[-1]
    anchor = points[0]
    hull = [anchor]
    inv = False

    if viz:
        # plot initial points
        plot_init(init, hull)

    while True:
        points.remove(anchor)
        max_angle = find_max_angle(anchor, points, inv)

        if max_angle == None and inv == False:
            inv = True
            max_angle = find_max_angle(anchor, points, inv)

        elif (max_angle == None) or (max_angle[1] == final and inv == True):
             # plot added edge
            if viz:
                hull.append(anchor)
                plot_final(init, hull)
            return hull
        
        points.append(anchor)
        anchor = max_angle[1]
        hull.append(anchor)
        # plot added edge
        if viz:
            plot_update(init, hull)

    return hull

def right_turn(a, b, c):
    """Returns the direction of the turn from a to b to c."""
    x1, y1 = a
    x2, y2 = b
    x3, y3 = c
    cross_product = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
    if cross_product >= 0:
        return False
    else:
        return True

def graham_scan(n, viz = True):
    """finds the convex hull of a set using gift-wrapping"""
    points = generate_points(n)

    points = sorted(points, key=lambda x: x[1])
    anchor = points[0]
    sorted_points = sort_by_angles(anchor, points[1:])
    hull = [anchor]

    # plot initial points
    if viz:
       plot_init(points, hull)

    for point in sorted_points:
        while len(hull) > 1 and right_turn(hull[-2], hull[-1], point):
            hull.pop()
            # plot deleted edge
            if viz:
                plot_update(points, hull)

        hull.append(point)
        if viz:
            plot_update(points, hull)

    if viz:
        plot_final(points, hull)

    return hull

def plot_init(points, hull):
    plt.scatter(*zip(*points))
    xs, ys = [*zip(*hull)]
    xs = list(xs)
    ys = list(ys)
    xs.append(xs[0])
    ys.append(ys[0])
    plt.plot(xs, ys, "r-")
    plt.pause(1)

def plot_update(points, hull):
    plt.clf()
    plt.scatter(*zip(*points))
    xs, ys = [*zip(*hull)]
    xs = list(xs)
    ys = list(ys)
    xs.append(xs[0])
    ys.append(ys[0])
    plt.plot(xs, ys, "r-")
    plt.pause(0.001)

def plot_final(points, hull):
    plt.clf()
    plt.scatter(*zip(*points))
    xs, ys = [*zip(*hull)]
    xs = list(xs)
    ys = list(ys)
    xs.append(xs[0])
    ys.append(ys[0])
    plt.plot(xs, ys, "r-")
    plt.show()


def timing(num_points):
    start_time = time.time()
    incremental(num_points, viz = False)
    print("Incremental Algorithm: --- %s seconds ---" % (time.time() - start_time))
    start_time = time.time()
    gift_wrapping(num_points, viz = False)
    print("Gift Wrapping Algorithm: --- %s seconds ---" % (time.time() - start_time))
    start_time = time.time()
    graham_scan(num_points, viz = False)
    print("Graham-Scan Algorithm: --- %s seconds ---" % (time.time() - start_time))
    
if __name__ == "__main__":
    globals()[sys.argv[1]](int(sys.argv[2]))