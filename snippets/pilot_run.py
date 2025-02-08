from aerialist.px4.drone_test import DroneTest
from aerialist.px4.obstacle import Obstacle
from testcase import TestCase
import time
import statistics
from datetime import datetime
import os
import shutil
import json
from utils import calculate_comfort
from typing import List, Dict

# create two new folders for storing .png and .ulg files
current_file_path = __file__
current_directory = os.path.dirname(current_file_path)
current_file_name = os.path.splitext(os.path.basename(current_file_path))[0]
current_time = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
original_log_directory = os.path.join(current_directory, "results/logs")
original_plot_directory = os.path.join(current_directory, "results")
new_folder_name = f"{current_time}"
new_log_path = os.path.join(original_log_directory, new_folder_name)
new_plot_path = os.path.join(original_plot_directory, new_folder_name)
os.makedirs(new_log_path, exist_ok=True)
os.makedirs(new_plot_path, exist_ok=True)

class ObstacleParam:
    def __init__(self, x: Dict[str, float], i: int) -> None:
        # position
        self.x = x[f"x{i}"]
        self.y = x[f"y{i}"]
        self.r = x[f"r{i}"]
        # size
        self.l = x[f"l{i}"]
        self.w = x[f"w{i}"]
        self.h = x[f"h{i}"]

        # other
        self.corners = None
        self.polygon = None

    def __str__(self):
        s = f"x={self.x}, y={self.y}, r={self.r}, l={self.l}, w={self.w}, h={self.h}"
        return s

def execute(x):
    obstacle_num = int(x["n"])
    scenario: List[Obstacle] = []
    scenario_param: List[ObstacleParam] = []
    for i in range(obstacle_num):
        # calculate the corners of each obstacle
        obstacle_param = ObstacleParam(x, i)
        scenario_param.append(obstacle_param)

        # create the obstacle
        position = Obstacle.Position(x=obstacle_param.x, y=obstacle_param.y, z=0, r=obstacle_param.r)
        size = Obstacle.Size(l=obstacle_param.l, w=obstacle_param.w, h=obstacle_param.h)
        obstacle = Obstacle(size, position)
        scenario.append(obstacle)
    case_study = x["case_study"]
    test = TestCase(case_study, scenario)
    simulation_time = 0
    min_distance = 0
    flight_trajectory = None
    try:
        start_time = time.time()
        flight_trajectory = test.execute()
        end_time = time.time()
        simulation_time = round(end_time - start_time, 2)
        min_distance = min(test.get_distances())
        test.plot()
        move_results()
    except Exception as e:
        print("Exception during test execution, skipping the test")
        print(e)
    max_jerk = calculate_comfort(new_log_path)
    return simulation_time, min_distance, max_jerk

def move_results():
    ulg_files = [f for f in os.listdir(original_log_directory) if f.endswith('.ulg')]
    latest_ulg = max(ulg_files, key=lambda f: os.path.getmtime(os.path.join(original_log_directory, f)))
    latest_ulg_path = os.path.join(original_log_directory, latest_ulg)
    shutil.move(latest_ulg_path, new_log_path)

    plot_files = [f for f in os.listdir(original_plot_directory) if f.endswith('.png')]
    latest_plot = max(plot_files, key=lambda f: os.path.getmtime(os.path.join(original_plot_directory, f)))
    latest_plot_path = os.path.join(original_plot_directory, latest_plot)
    shutil.move(latest_plot_path, new_plot_path)


if __name__ == "__main__":
    case_study1 = DroneTest.from_yaml("case_studies/mission1.yaml")
    case_study2 = DroneTest.from_yaml("case_studies/mission2.yaml")
    case_study3 = DroneTest.from_yaml("case_studies/mission3.yaml")

    x0 = {'case_study': case_study1, 'n': 1, 'index': 0, 'x0': 5.3515303028707955, 'y0': 22.394779942080447, 'r0': 67.53701554273161, 'l0': 7.768550509158968, 'w0': 7.145600973708801, 'h0': 16.537064729664817, 'x1': -5.324137191513621, 'y1': 20.19070699851135, 'r1': 0.5301395494539385, 'l1': 2.8408094033812965, 'w1': 10.182411183519063, 'h1': 23.83241137571917, 'x2': 12.997622552723026, 'y2': 36.58754685927904, 'r2': 74.96150424635272, 'l2': 6.346582431219196, 'w2': 10.059949367836593, 'h2': 21.970539168911706}
    x1 = {'case_study': case_study1, 'n': 2, 'index': 2, 'x0': 2.0175229201393776, 'y0': 26.875451150964235, 'r0': 9.33102286775148, 'l0': 14.449264219237314, 'w0': 15.075550645862426, 'h0': 21.45233470557975, 'x1': -36.51872900641356, 'y1': 26.573714984942608, 'r1': 9.464155679218088, 'l1': 16.0351406073529, 'w1': 9.058014891282662, 'h1': 23.167648987107984, 'x2': 28.33883087918801, 'y2': 15.488713798472638, 'r2': 39.73211965332544, 'l2': 10.304560275847091, 'w2': 6.82981265284774, 'h2': 21.263858537440882}
    x2 = {'case_study': case_study1, 'n': 2, 'index': 1, 'x0': -27.017107534768385, 'y0': 16.26154997606058, 'r0': 16.742468980011843, 'l0': 13.450210483796583, 'w0': 14.60140205304471, 'h0': 23.529408828545627, 'x1': 28.261102810903424, 'y1': 11.185749454627663, 'r1': 68.47525234490887, 'l1': 9.688556121652143, 'w1': 7.53555447263775, 'h1': 18.804365331343753, 'x2': -38.174216456042565, 'y2': 30.2486180086458, 'r2': 66.01227564104838, 'l2': 18.87206052629683, 'w2': 2.735353412430042, 'h2': 24.988998032041152}
    x3 = {'case_study': case_study2, 'n': 3, 'index': 0, 'x0': -32.63285415879732, 'y0': 35.32298606773288, 'r0': 60.18582156585002, 'l0': 12.458783791489479, 'w0': 14.797197525997408, 'h0': 18.582182172071175, 'x1': -3.9824395598363367, 'y1': 34.48456022918131, 'r1': 19.897850820675956, 'l1': 3.1499539810965462, 'w1': 12.660796258693864, 'h1': 17.677998825094605, 'x2': -7.915418780165012, 'y2': 31.469411605126798, 'r2': 55.21014589363627, 'l2': 14.55858878370439, 'w2': 19.116011208317243, 'h2': 24.1621098139601}
    x4 = {'case_study': case_study2, 'n': 1, 'index': 2, 'x0': -29.184341898184357, 'y0': 14.988507555315934, 'r0': 44.35490830512759, 'l0': 5.9520545847557464, 'w0': 16.12454827343452, 'h0': 15.618961564138905, 'x1': -1.8738246424221643, 'y1': 15.738116857061744, 'r1': 26.787423275601526, 'l1': 15.237994534708218, 'w1': 17.375734296660035, 'h1': 24.46953336423514, 'x2': -7.789961376908927, 'y2': 12.541794183677041, 'r2': 63.06050468329069, 'l2': 6.384893659241607, 'w2': 12.922047420751932, 'h2': 24.17301913724512}
    x5 = {'case_study': case_study2, 'n': 3, 'index': 1, 'x0': -25.355796298732884, 'y0': 10.224003425567101, 'r0': 53.921835826168184, 'l0': 19.37750569693075, 'w0': 10.531409861358247, 'h0': 22.094006254392994, 'x1': 12.724021909719944, 'y1': 17.264154809479056, 'r1': 59.931527905932114, 'l1': 9.660101079124136, 'w1': 13.853580481007324, 'h1': 15.9083189443367, 'x2': 24.143964382716945, 'y2': 39.19978592330955, 'r2': 38.173309805435736, 'l2': 4.687523547311433, 'w2': 3.951202943855848, 'h2': 15.724616360359938}
    x6 = {'case_study': case_study3, 'n': 1, 'index': 1, 'x0': -0.5149768490078728, 'y0': 38.347751441077264, 'r0': 85.528724613963, 'l0': 19.28260911658024, 'w0': 5.691282322276327, 'h0': 16.565912620373066, 'x1': -19.846891021100053, 'y1': 16.57874963575602, 'r1': 81.55118536553631, 'l1': 16.950314847770912, 'w1': 12.883217609737454, 'h1': 18.770991123950164, 'x2': 9.39467565019114, 'y2': 36.34844582168117, 'r2': 40.72192903336384, 'l2': 17.75807700531861, 'w2': 4.35353245171749, 'h2': 19.862879529904635}
    x7 = {'case_study': case_study3, 'n': 2, 'index': 0, 'x0': -22.58368725004267, 'y0': 23.22299310677159, 'r0': 53.06365579835172, 'l0': 4.347547891922423, 'w0': 18.56553907125236, 'h0': 24.859174158896124, 'x1': -11.062236209577836, 'y1': 19.144252645471134, 'r1': 43.67721524284974, 'l1': 11.344120391516622, 'w1': 3.9112380884956544, 'h1': 19.6781774801141, 'x2': 6.615138595752235, 'y2': 37.313385145634044, 'r2': 77.38865439751933, 'l2': 5.075387284833636, 'w2': 17.922713007130596, 'h2': 18.768681122229466}
    x8 = {'case_study': case_study3, 'n': 2, 'index': 0, 'x0': 22.633375724553233, 'y0': 37.67615813649256, 'r0': 57.87788785188628, 'l0': 8.303533779299142, 'w0': 5.0884294648929025, 'h0': 18.63863677220829, 'x1': -37.18524542869669, 'y1': 21.08078591152278, 'r1': 68.28528460466784, 'l1': 15.66438699582181, 'w1': 9.434369185348881, 'h1': 24.35783386381792, 'x2': -36.96516098282207, 'y2': 23.828651042328357, 'r2': 13.640380039106933, 'l2': 11.08321235405768, 'w2': 19.15346087768347, 'h2': 17.300182178440455}
    x9 = {'case_study': case_study3, 'n': 2, 'index': 0, 'x0': -22.58368725004267, 'y0': 23.22299310677159, 'r0': 53.06365579835172, 'l0': 4.938166450425461, 'w0': 18.56553907125236, 'h0': 24.76492190901226, 'x1': -11.062236209577836, 'y1': 19.144252645471134, 'r1': 43.67721524284974, 'l1': 11.344120391516622, 'w1': 3.9112380884956544, 'h1': 19.6781774801141, 'x2': 6.615138595752235, 'y2': 37.313385145634044, 'r2': 77.38865439751933, 'l2': 5.075387284833636, 'w2': 17.922713007130596, 'h2': 18.768681122229466}
    scenarios = [x0, x1, x2, x3, x4, x5, x6, x7, x8, x9]
    results = {}
    for i in range(10):  # 10 scenarios
        result = {}
        performance_list = []
        safety_list = []
        comfort_list = []
        for j in range(10):  # 10 repetitions
            performance, safety, comfort = execute(scenarios[i])
            performance_list.append(performance)
            safety_list.append(safety)
            comfort_list.append(comfort)
        performance_statistics = {'mean': statistics.mean(performance_list), 'median': statistics.median(performance_list), 'std': statistics.stdev(performance_list)}
        safety_statistics = {'mean': statistics.mean(safety_list), 'median': statistics.median(safety_list), 'std': statistics.stdev(safety_list)}
        comfort_statistics = {'mean': statistics.mean(comfort_list), 'median': statistics.median(comfort_list), 'std': statistics.stdev(comfort_list)}
        result["performance_statistics"] = performance_statistics
        result["safety_statistics"] = safety_statistics
        result["comfort_statistics"] = comfort_statistics

        results[f"scenario{i}"] = result

    with open(os.path.join(new_log_path, "results.json"), "w") as file:
        json.dump(results, file, indent=4)
