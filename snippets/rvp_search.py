import copy
import os
import re
import yaml
import shutil
import statistics
import openpyxl
from openpyxl import Workbook
from typing import List, Dict
from pymoo.termination import get_termination
from pymoo.core.problem import ElementwiseProblem
from pymoo.core.mixed import MixedVariableDuplicateElimination
from genetic_algorithm import MixedVariableGA, MixedVariableMating, RankAndCrowding
from pymoo.core.variable import Real, Integer, BoundedVariable
from pymoo.util.display.multi import MultiObjectiveOutput
from pymoo.optimize import minimize
from shapely.geometry import Polygon, Point
from aerialist.px4.drone_test import DroneTest
from aerialist.px4.obstacle import Obstacle
from testcase import TestCase
from itertools import combinations
import time
from datetime import datetime
import logging
from utils import calculate_accuracy, calculate_comfort, calculate_corners

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

# create the log file
system_log_path = os.path.join(new_log_path, f"{current_file_name}_{current_time}.log")
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s  %(filename)s : %(levelname)s  %(message)s',
                    datefmt='%Y-%m-%d %A %H:%M:%S',
                    filename=system_log_path,
                    filemode='w')
logger = logging.getLogger(__name__)


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


class RvpProblem(ElementwiseProblem):
    min_size = Obstacle.Size(2, 2, 15)
    max_size = Obstacle.Size(20, 20, 25)
    # fixed area: -40 < x < 30, 10 < y < 40
    min_position = Obstacle.Position(-40, 10, 0, 0)
    max_position = Obstacle.Position(30, 40, 0, 90)

    def __init__(self, config, target_pattern, sheet):
        self.config = config
        self.case_study_file = self.config["case_study_file"]
        self.case_study = DroneTest.from_yaml(self.case_study_file)
        self.target_pattern = target_pattern
        self.sheet = sheet

        vars: Dict[str, BoundedVariable] = {}
        vars["n"] = Integer(bounds=(self.config["num_of_obstacles"][0], self.config["num_of_obstacles"][1]))
        vars["index"] = Integer(bounds=(self.config["index"][0], self.config["index"][1]))
        if self.config["MR_name"] == "decrease":
            vars["scale"] = Real(bounds=(self.config["scale"][0], self.config["scale"][1]))
        for i in range(self.config["num_of_obstacles"][1]):
            vars[f"x{i}"] = Real(bounds=(self.min_position.x, self.max_position.x))
            vars[f"y{i}"] = Real(bounds=(self.min_position.y, self.max_position.y))
            vars[f"r{i}"] = Real(bounds=(self.min_position.r, self.max_position.r))
            vars[f"l{i}"] = Real(bounds=(self.min_size.l, self.max_size.l))
            vars[f"w{i}"] = Real(bounds=(self.min_size.w, self.max_size.w))
            vars[f"h{i}"] = Real(bounds=(self.min_size.h, self.max_size.h))

        super().__init__(vars=vars, n_obj=4, n_ieq_constr=1)

    def _evaluate(self, x: Dict[str, float], out: Dict[str, List[float]], *args, **kwargs):
        logger.info("x: %s", x)
        obstacle_num = int(x["n"])
        src_scenario: List[Obstacle] = []
        fwp_scenario: List[Obstacle] = []
        src_scenario_param: List[ObstacleParam] = []
        fwp_scenario_param: List[ObstacleParam] = []
        for i in range(obstacle_num):
            # calculate the corners of each obstacle
            obstacle_param = ObstacleParam(x, i)
            calculate_corners(obstacle_param)
            obstacle_param.polygon = Polygon(obstacle_param.corners)
            src_scenario_param.append(obstacle_param)
            fwp_scenario_param.append(obstacle_param)

            # create the obstacle
            position = Obstacle.Position(x=obstacle_param.x, y=obstacle_param.y, z=0, r=obstacle_param.r)
            size = Obstacle.Size(l=obstacle_param.l, w=obstacle_param.w, h=obstacle_param.h)
            obstacle = Obstacle(size, position)
            src_scenario.append(obstacle)

        # prepare the followup scenario
        index = int(x["index"])
        out["G"] = [index - (
                obstacle_num - 1)]  # Constraint: 0 <= index <= n - 1, since we cannot modify an obstacle that does not exist
        if index >= obstacle_num:
            index = 0
        if self.config["MR_name"] == "decrease":
            fwp_scenario, fwp_scenario_param = self.decrease(src_scenario, src_scenario_param, index, x["scale"])
        else:
            fwp_scenario, fwp_scenario_param = self.remove(src_scenario, src_scenario_param, index)

        logger.info("src_scenario: %s \n fwp_scenario: %s", [str(o) for o in src_scenario_param], [str(o) for o in fwp_scenario_param])

        # execute the source & followup scenario
        metrics = {
            "source": {
                "performance": [],
                "safety": [],
                "comfort": [],
            },
            "followup": {
                "performance": [],
                "safety": [],
                "comfort": [],
            }
        }
        self.run_pair(src_scenario, src_scenario_param, fwp_scenario, fwp_scenario_param, metrics, repetitions=self.config["repetitions"])

        avg_metrics = {
            scenario: {metric: statistics.mean(values) for metric, values in data.items()}
            for scenario, data in metrics.items()
        }
        logger.info("Average metrics: %s", avg_metrics)

        # the execution time is expected to decrease if the performance requirement must be satisfied
        performance_diff = avg_metrics["followup"]["performance"] - avg_metrics["source"]["performance"]
        # the min distance is expected to increase if the safety requirement must be satisfied
        safety_diff = avg_metrics["source"]["safety"] - avg_metrics["followup"]["safety"]
        # the max jerk is expected to decrease if the comfort requirement must be satisfied
        comfort_diff = avg_metrics["followup"]["comfort"] - avg_metrics["source"]["comfort"]

        # another fitness: we calculate the area of overlaps and minimize it
        overlap_area = 0
        polygons = [param.polygon for param in src_scenario_param]
        for poly1, poly2 in combinations(polygons, 2):  # All pairs
            if poly1.intersects(poly2):
                intersection = poly1.intersection(poly2)
                area = intersection.area
                overlap_area += area
        logger.info("overlapped area: %f", overlap_area)

        out["F"] = [
            overlap_area,  # we minimize the overlap num to ensure scenario plausibility
            performance_diff if self.target_pattern[0] is True else -1.0 * performance_diff,
            safety_diff if self.target_pattern[1] is True else -1.0 * safety_diff,
            comfort_diff if self.target_pattern[2] is True else -1.0 * comfort_diff,
        ]

        logger.info("Output F: %s, Plot file: %s", str(out["F"]), self.get_latest_plot())

        # record if the RVP is satisfied
        if self.judge_satisfaction([performance_diff, safety_diff, comfort_diff]):
            logger.info("Find a satisfied RVP: %s, plot file: %s", self.target_pattern, self.get_latest_plot())
            mission_id = re.search(r"mission(\d+)", self.config["case_study_file"], re.IGNORECASE)
            mission_name = f"Mission {mission_id.group(1)}"
            new_row = [mission_name, self.config["MR_name"], str(self.target_pattern), str(avg_metrics), self.get_latest_plot()]
            self.sheet.append(new_row)

    @staticmethod
    def decrease(src_scenario, src_scenario_param, index, scale):
        fwp_scenario = copy.deepcopy(src_scenario)
        fwp_scenario_param = copy.deepcopy(src_scenario_param)

        new_l = fwp_scenario[index].size.l * scale
        new_w = fwp_scenario[index].size.w * scale
        new_h = fwp_scenario[index].size.h * scale
        fwp_scenario[index].size = Obstacle.Size(l=new_l, w=new_w, h=new_h)
        fwp_scenario_param[index].l = new_l
        fwp_scenario_param[index].w = new_w
        fwp_scenario_param[index].h = new_h

        return fwp_scenario, fwp_scenario_param

    @staticmethod
    def remove(src_scenario, src_scenario_param, index):
        fwp_scenario = copy.deepcopy(src_scenario)
        fwp_scenario_param = copy.deepcopy(src_scenario_param)

        fwp_scenario.pop(index)
        fwp_scenario_param.pop(index)

        return fwp_scenario, fwp_scenario_param

    def run_pair(self, src_scenario, src_scenario_param, fwp_scenario, fwp_scenario_param, metrics, repetitions=1):
        for _ in range(repetitions):
            performance, _, trajectory_2d = self.execute(src_scenario)
            comfort = calculate_comfort(new_log_path)
            safety = self.calculate_min_distance(src_scenario_param, trajectory_2d)
            metrics["source"]["performance"].append(performance)
            metrics["source"]["safety"].append(safety)
            metrics["source"]["comfort"].append(comfort)

        for _ in range(repetitions):
            performance, _, trajectory_2d = self.execute(fwp_scenario)
            comfort = calculate_comfort(new_log_path)
            safety = self.calculate_min_distance(fwp_scenario_param, trajectory_2d)
            metrics["followup"]["performance"].append(performance)
            metrics["followup"]["safety"].append(safety)
            metrics["followup"]["comfort"].append(comfort)

    def execute(self, scenario):
        test = TestCase(self.case_study, scenario)
        simulation_time = 999
        min_distance = 999
        trajectory_2d = None
        try:
            start_time = time.time()
            trajectory = test.execute()
            trajectory_2d = [(position.x, position.y) for position in trajectory.positions]
            end_time = time.time()
            simulation_time = round(end_time - start_time, 2)
            min_distance = min(test.get_distances())
            test.plot()
            self.move_results()
        except Exception as e:
            logger.error(e)

        return simulation_time, min_distance, trajectory_2d

    @staticmethod
    def calculate_min_distance(src_scenario_param, trajectory_2d):
        min_distance = 999
        if trajectory_2d is None:
            return min_distance
        else:
            for x, y in trajectory_2d:
                point = Point(x, y)
                for obstacle_param in src_scenario_param:
                    distance = point.distance(obstacle_param.polygon)
                    min_distance = min(min_distance, distance)
            return min_distance

    def judge_satisfaction(self, diff):
        count = 0
        for d, target in zip(diff, self.target_pattern):
            if (d < 0 and target) or (d > 0 and not target):
                count += 1
        if count == len(self.target_pattern):
            return True
        return False

    def move_results(self):
        ulg_files = [f for f in os.listdir(original_log_directory) if f.endswith('.ulg')]
        latest_ulg = max(ulg_files, key=lambda f: os.path.getmtime(os.path.join(original_log_directory, f)))
        latest_ulg_path = os.path.join(original_log_directory, latest_ulg)
        shutil.move(latest_ulg_path, new_log_path)

        plot_files = [f for f in os.listdir(original_plot_directory) if f.endswith('.png')]
        latest_plot = max(plot_files, key=lambda f: os.path.getmtime(os.path.join(original_plot_directory, f)))
        latest_plot_path = os.path.join(original_plot_directory, latest_plot)
        shutil.move(latest_plot_path, new_plot_path)

    def get_latest_plot(self):
        plot_files = [f for f in os.listdir(new_plot_path) if f.endswith('.png')]
        latest_file = max(plot_files, key=lambda f: os.path.getmtime(os.path.join(new_plot_path, f)))
        return latest_file


class RvpGenerator:
    def __init__(self, config, target_pattern):
        self.config = config
        self.target_pattern = target_pattern

        self.result_path = os.path.join(new_log_path, 'rvp_results.xlsx')
        if not os.path.exists(self.result_path):
            self.workbook = Workbook()
            self.sheet = self.workbook.active
            self.sheet.title = "RVP Results"
            headers = ["Mission Name", "MR Name", "Pattern", "Average Metrics", "Plot File"]
            self.sheet.append(headers)
        else:
            self.workbook = openpyxl.load_workbook(self.result_path)
            self.sheet = self.workbook.active

        self.problem = RvpProblem(self.config, self.target_pattern, self.sheet)

    def generate(self):
        algorithm = MixedVariableGA(pop_size=self.config["pop_size"],
                                    n_offsprings=self.config["num_of_offsprings"],
                                    output=MultiObjectiveOutput(),
                                    survival=RankAndCrowding(),
                                    mating=MixedVariableMating(eliminate_duplicates=MixedVariableDuplicateElimination()),
                                    )
        termination = get_termination("n_gen", self.config["max_gen"])
        res = minimize(self.problem, algorithm, termination, verbose=1)

        self.workbook.save(self.result_path)


if __name__ == "__main__":
    with open("config.yaml") as f:
        config = yaml.safe_load(f)
    target_pattern = config["target_pattern"]
    for i in range(2):  # 2 MRs
        if i == 0:
            config["MR_name"] = "remove"
            config["num_of_obstacles"] = [2, 3]
        else:
            config["MR_name"] = "decrease"
            config["num_of_obstacles"] = [1, 3]
        for j in range(3):  # 3 missions
            config["case_study_file"] = f"case_studies/mission{j + 1}.yaml"
            for k in range(len(target_pattern)):
                logger.info("Mission File: %s, target_pattern: %s", config["case_study_file"], str(target_pattern[k]))
                generator = RvpGenerator(config, target_pattern[k])
                generator.generate()
