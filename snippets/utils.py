import math
import random
import sys
import matplotlib.pyplot as pl
import matplotlib.patches as patches
import matplotlib as mpl
import os
import pandas
import subprocess
import numpy as np
import json

def random_rectangle(center_x, center_y, radius, eps=0.1):
    """Create random rectangle (x, y, l, w, r) inside a given circle."""
    min_length = eps * radius
    max_length = (1 - eps) * radius
    length = random.uniform(min_length, max_length) # half length, since radius is half diameter
    width = math.sqrt(radius ** 2 - length ** 2) # half width, since radius is half diameter
    rotation = random.uniform(0, 90) # in degrees
    scalar = 0.999 # make it a bit smaller, as we discussed
    return (center_x, center_y, scalar * length * 2, scalar * width * 2, rotation)

def get_subrectangles(x, y, l, w, r, count=4):
    """Subdivide the rectangle in x and y direction count times"""
    r_radian = math.pi * r / 180
    x1 = x - l / 2
    x2 = x + l / 2
    xmin = min([x1, x2])
    xmax = max([x1, x2])
    xdiff = xmax - xmin

    y1 = y - w / 2
    y2 = y + w / 2
    ymin = min([y1, y2])
    ymax = max([y1, y2])
    ydiff = ymax - ymin

    rectangles = []
    for i in range(count):
        for j in range(count):
            rxmin = xmin + i * xdiff / count
            rx = rxmin + xdiff / (2 * count)
            rymin = ymin + j * ydiff / count
            ry = rymin + (ydiff / (2 * count))
            rl = l / count
            rw = w / count

            dx = rx - x
            dy = ry - y

            # Rotation matrix c, -s; s, c
            newdx = math.cos(r_radian) * dx - math.sin(r_radian) * dy
            newdy = math.sin(r_radian) * dx + math.cos(r_radian) * dy

            rectangles.append((x + newdx, y + newdy, rl, rw, r))

    return rectangles


def single_circle_coverage(x, y, l, w, r):
    """Create a circle (center_x, center_y, radius) that encompases a given
    rectangle (x, y, l, w, r)"""
    return (x, y, math.sqrt((l/2) ** 2 + (w/2) ** 2))


def circle_coverage(x, y, l, w, r, subdivision_count=4):
    """Create subdivision_count^2 number of circles (center_x, center_y, radius)
    that together encompass a given rectangle (x, y, l, w, r) with r in
    degrees indicating counterclockwise rotation"""

    circles = []
    for subrectangle in get_subrectangles(x, y, l, w, r, subdivision_count):
        circles.append(single_circle_coverage(*subrectangle))

    return circles

def random_nonintersecting_circle(center_x, center_y, upper_b, lower_b, left_b, right_b, other_circles):
    """Given other_circles (as a list of (cx, cy, radius)), find the largest circle
    that does not intersect with other circles"""
    radius = sys.float_info.max
    for (other_x, other_y, other_radius) in other_circles:
        distance = math.sqrt((other_x - center_x)**2 + (other_y - center_y)**2)
        boundary_distance = get_boundary_distance(center_x, center_y, upper_b, lower_b, left_b, right_b)
        radius = min([radius, distance - other_radius, boundary_distance])
    if radius <= 0:
        return None
    else:
        coeff = random.uniform(0.5, 0.9)
        return center_x, center_y, coeff * radius

def random_nonintersecting_rectangle(center_x, center_y, upper_b, lower_b, left_b, right_b, other_rectangles, subdivision_count=4):
    """Given other_rectangles (as a list of (x, y, l, w, r)), return a random rectangle
    inside the largest circle that does not intersect with the circles that cover the other rectangles."""
    all_other_circles = []
    print(other_rectangles)
    for other_rectangle in other_rectangles:
        all_other_circles += circle_coverage(*other_rectangle, subdivision_count)
    circle = random_nonintersecting_circle(center_x, center_y, upper_b, lower_b, left_b, right_b, all_other_circles)
    if circle is not None:
        return random_rectangle(*circle)
    else:
        return None

def get_boundary_distance(center_x, center_y, upper_b, lower_b, left_b, right_b):
    upper_distance = upper_b - center_y
    lower_distance = center_y - lower_b
    left_distance = center_x - left_b
    right_distance = right_b - center_x
    return min([upper_distance, lower_distance, left_distance, right_distance])

def plot_rectangle(rectangles):
    for (x, y, l, w, r) in rectangles:
        pl.plot([x], [y], color="#FF000030", marker='o', markersize=10)
        rect = patches.Rectangle((x - l / 2, y - w / 2), l, w, color="#FF000030", alpha=0.10, angle=r, rotation_point='center')
        ax = pl.gca()
        ax.add_patch(rect)
    pl.figure()
    pl.plot()
    pl.xlim([-10, 150])
    pl.ylim([-10, 150])
    pl.show()

def calculate_corners(obstacle_param):
    half_l = obstacle_param.l / 2
    half_w = obstacle_param.w / 2
    corners = [
        (obstacle_param.x + half_l * math.cos(math.radians(obstacle_param.r)) - half_w * math.sin(
            math.radians(obstacle_param.r)),
         obstacle_param.y + half_l * math.sin(math.radians(obstacle_param.r)) + half_w * math.cos(
             math.radians(obstacle_param.r))),
        (obstacle_param.x - half_l * math.cos(math.radians(obstacle_param.r)) - half_w * math.sin(
            math.radians(obstacle_param.r)),
         obstacle_param.y - half_l * math.sin(math.radians(obstacle_param.r)) + half_w * math.cos(
             math.radians(obstacle_param.r))),
        (obstacle_param.x + half_l * math.cos(math.radians(obstacle_param.r)) + half_w * math.sin(
            math.radians(obstacle_param.r)),
         obstacle_param.y + half_l * math.sin(math.radians(obstacle_param.r)) - half_w * math.cos(
             math.radians(obstacle_param.r))),
        (obstacle_param.x - half_l * math.cos(math.radians(obstacle_param.r)) + half_w * math.sin(
            math.radians(obstacle_param.r)),
         obstacle_param.y - half_l * math.sin(math.radians(obstacle_param.r)) - half_w * math.cos(
             math.radians(obstacle_param.r)))
    ]

    def angle_from_centroid(point):
        return np.arctan2(point[1] - obstacle_param.y, point[0] - obstacle_param.x)

    sorted_corners = sorted(corners, key=angle_from_centroid)  # otherwise may occur self-intersection
    obstacle_param.corners = sorted_corners

def calculate_comfort(ulg_folder):  # measured by max Jerk (https://en.wikipedia.org/wiki/Jerk_(physics))
    ulg_files = [f for f in os.listdir(ulg_folder) if f.endswith('.ulg')]
    ulg_file = max(ulg_files, key=lambda f: os.path.getmtime(os.path.join(ulg_folder, f)))
    topic = "vehicle_acceleration"
    ulg_file_path = os.path.join(ulg_folder, ulg_file)
    csv_file_prefix = ulg_file.split('.')[0]  # File prefix without extension
    csv_file_path = ulg_folder + f'/{csv_file_prefix}_{topic}_0.csv'
    command = f"ulog2csv -m '{topic}' {ulg_file_path}"
    subprocess.run(command, check=True, shell=True)
    df = pandas.read_csv(csv_file_path)
    # battery_list.append(1 - df.iloc[-1, 7])
    time = df['timestamp'].values  # Timestamps in seconds
    acc_x = df['xyz[0]'].values  # Acceleration in x-axis (m/s²)
    acc_y = df['xyz[1]'].values  # Acceleration in y-axis (m/s²)
    acc_z = df['xyz[2]'].values  # Acceleration in z-axis (m/s²)

    # Compute jerk (rate of change of acceleration) for each axis
    jerk_x = np.gradient(acc_x, time)  # Jerk in x-axis (m/s³)
    jerk_y = np.gradient(acc_y, time)  # Jerk in y-axis (m/s³)
    jerk_z = np.gradient(acc_z, time)  # Jerk in z-axis (m/s³)

    # Compute the total jerk magnitude at each timestamp
    jerk_magnitude = np.sqrt(jerk_x ** 2 + jerk_y ** 2 + jerk_z ** 2)

    # Find the maximum jerk magnitude
    max_jerk = np.max(jerk_magnitude)

    # delete the csv file after calculation
    os.remove(csv_file_path)

    return max_jerk

def calculate_accuracy(ulg_folder, yaml_file_name):
    # read the .plan file and get the target global position
    plan_file_path = os.path.splitext(yaml_file_name)[0] + ".plan"
    with open(plan_file_path, 'r') as f:
        mission_plan = json.load(f)
    target_lat = mission_plan["mission"]["items"][-1]["params"][4]
    target_lon = mission_plan["mission"]["items"][-1]["params"][5]

    # get the actual global position
    ulg_files = [f for f in os.listdir(ulg_folder) if f.endswith('.ulg')]
    ulg_file = max(ulg_files, key=lambda f: os.path.getmtime(os.path.join(ulg_folder, f)))
    topic = "vehicle_global_position"
    ulg_file_path = os.path.join(ulg_folder, ulg_file)
    csv_file_prefix = ulg_file.split('.')[0]  # File prefix without extension
    csv_file_path = ulg_folder + f'/{csv_file_prefix}_{topic}_0.csv'
    command = f"ulog2csv -m '{topic}' {ulg_file_path}"
    subprocess.run(command, check=True, shell=True)
    df = pandas.read_csv(csv_file_path)
    last_row = df.iloc[-1]
    actual_lat = last_row["lat"]
    actual_lon = last_row["lon"]

    distance = math.sqrt((target_lat - actual_lat) ** 2 + (target_lon - actual_lon) ** 2)

    # delete the csv file after calculation
    os.remove(csv_file_path)
    os.remove(ulg_file_path)
    return distance




if __name__ == "__main__":
    from rvp_search import ObstacleParam
    from shapely.geometry import Polygon
    import matplotlib.pyplot as plt
    from itertools import combinations

    x = {'n': 3, 'index': 2, 'scale': 0.5417220027684608, 'x0': 0.5, 'y0': 0.5, 'r0': 0, 'l0': 1, 'w0': 1, 'h0': 17.208782781496268, 'x1': 1.5, 'y1': 1.5, 'r1': 0, 'l1': 0.99, 'w1': 0.99, 'h1': 21.413586672726325, 'x2': 1, 'y2': 1, 'r2': 0, 'l2': 1, 'w2': 1, 'h2': 17.478069307739847}

    o1 = ObstacleParam(x, 0)
    o2 = ObstacleParam(x, 1)
    o3 = ObstacleParam(x, 2)
    calculate_corners(o1)
    calculate_corners(o2)
    calculate_corners(o3)
    o1.polygon = Polygon(o1.corners)
    o2.polygon = Polygon(o2.corners)
    o3.polygon = Polygon(o3.corners)
    polygons = [o1.polygon]

    total_overlap_area = 0
    overlap_num = 0
    for poly1, poly2 in combinations(polygons, 2):  # All pairs (o1-o2, o1-o3, o2-o3)
        if poly1.intersects(poly2):
            intersection = poly1.intersection(poly2)
            area = intersection.area
            total_overlap_area += area
            overlap_num += 1
    print("overlapped num: ", overlap_num)
    print("overlapped area: ", total_overlap_area)
    # --- Plotting ---
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot only the outlines of the obstacles
    for idx, poly in enumerate(polygons):
        x_coords, y_coords = poly.exterior.xy
        ax.plot(x_coords, y_coords, color='blue', linewidth=2, label=f'Obstacle {idx + 1}')

        # Annotate the center of each obstacle
        centroid = poly.centroid
        ax.annotate(f"O{idx + 1}", (centroid.x, centroid.y), ha='center', va='center', fontsize=10, weight='bold')

    # Set plot limits based on obstacle coordinates
    all_coords = [point for poly in polygons for point in poly.exterior.coords]
    x_vals, y_vals = zip(*all_coords)
    ax.set_xlim(min(x_vals) - 10, max(x_vals) + 10)
    ax.set_ylim(min(y_vals) - 10, max(y_vals) + 10)

    # Formatting
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend(loc='upper right')
    plt.title("Obstacle Outlines")

    plt.show()
    # print(explain_validity(o.polygon))
    # # Plotting to visualize
    # x, y = zip(*o.corners)
    # plt.figure(figsize=(6, 6))
    # plt.plot(x + (x[0],), y + (y[0],), marker='o')  # Close the polygon loop
    # # plt.scatter(*(-6.3391, 32.2536), color='red', label='Self-intersection')  # Mark the intersection point
    # plt.legend()
    # plt.title("Polygon Visualization with Self-Intersection")
    # plt.grid(True)
    # plt.show()
