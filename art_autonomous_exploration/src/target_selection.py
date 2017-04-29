#!/usr/bin/env python
from __future__ import division

import math
import random
import time

import numpy
import rospy

from brushfires import Brushfires
from path_planning import PathPlanning
from topology import Topology
from utilities import OgmOperations
from utilities import Print
from utilities import RvizHandler


# Class for selecting the next best target
class TargetSelection(object):
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method
        if self.method_is_cost_based():
            from robot_perception import RobotPerception
            self.robot_perception = RobotPerception()
            self.cost_based_properties = rospy.get_param("cost_based_properties")

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()

    def method_is_cost_based(self):
        return self.method == 'cost_based'

    # TODO: init_ogm -> ogm
    def selectTarget(self, init_ogm, coverage, robot_pose, origin, resolution, force_random=False):
        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the following tools: ogm_limits, Brushfire field,
        # OGM skeleton, topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)

        # Visualization of topological nodes
        vis_nodes = [[n[0] * resolution + origin['x'], n[1] * resolution + origin['y']] for n in nodes]
        RvizHandler.printMarker(
            vis_nodes,
            1,  # Type: Arrow
            0,  # Action: Add
            "map",  # Frame
            "art_topological_nodes",  # Namespace
            [0.3, 0.4, 0.7, 0.5],  # Color RGBA
            0.1  # Scale
        )

        if self.method == 'random' or force_random:
            return self.selectRandomTarget(ogm, coverage, brush)
        elif self.method_is_cost_based():
            g_robot_pose = self.robot_perception.getGlobalCoordinates([robot_pose['x_px'], robot_pose['y_px']])

            class MapContainer:
                def __init__(self):
                    self.skeleton = skeleton
                    self.coverage = coverage
                    self.ogm = ogm
                    # TODO: filter nodes elsewhere.
                    self.nodes = [node for node in nodes if TargetSelection.is_good(node, ogm, coverage, brush)]
                    self.xy_g = [robot_pose['x_px'] - origin['x'] / resolution,
                                 robot_pose['y_px'] - origin['y'] / resolution]
                    self.theta = robot_pose['th']

                @staticmethod
                def create_path(path_target):
                    return self.path_planning.createPath(g_robot_pose, path_target, resolution)

            target = self.select_by_cost(MapContainer())
            return target
        else:
            assert False, "Invalid target_selector method."

    @staticmethod
    def selectRandomTarget(ogm, coverage, brushogm):
        # The next target in pixels
        tinit = time.time()
        while True:
            x_rand = random.randint(0, ogm.shape[0] - 1)
            y_rand = random.randint(0, ogm.shape[1] - 1)
            if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and brushogm[x_rand][y_rand] > 5:
                Print.art_print("Select random target time: " + str(time.time() - tinit), Print.ORANGE)
                return x_rand, y_rand

    @staticmethod
    def is_good(target, ogm, coverage, brush):
        x, y = target
        return ogm[x][y] < 50 and coverage[x][y] < 50 and brush[x][y] > 5

    def select_by_cost(self, map_info):
        tinit = time.time()
        numpy.set_printoptions(precision=3, threshold=numpy.nan, suppress=True)  # TODO:del
        nodes, paths, topo_costs = self.choose_best_nodes(map_info)
        if not nodes:
            return -1, -1

        best_path_idx = self.weight_costs(
            topo_costs,
            [self.distance_cost(path, map_info.xy_g) for path in paths],  # Distance
            [self.coverage_cost(path, map_info.coverage) for path in paths],  # Coverage
            [self.rotation_cost(path, map_info.xy_g, map_info.theta) for path in paths]  # Rotational
        ).argmax()
        assert paths[best_path_idx]
        target = nodes[best_path_idx]

        Print.art_print("Best: {}".format(best_path_idx), Print.BLUE)
        Print.art_print("Select {} target time: {}".format(self.method, time.time() - tinit), Print.ORANGE)
        return target

    def choose_best_nodes(self, map_info):
        # Since path planning takes a lot of time for many nodes we should reduce the possible result to the nodes
        # with the best distance and topological costs.
        topo_costs = [self.topo_cost(node, map_info.ogm) for node in map_info.nodes]
        best_nodes_idx = self.weight_costs(
            # Use simple distance function without coeff. An alternative would be to use self.distance_coeff.
            [self.distance(node, map_info.xy_g) for node in map_info.nodes],
            topo_costs,
        ).argsort()[::-1]  # We need descending order since we now want to maximize.

        count = 0
        paths = []
        nodes_new = []
        topo_costs_new = []
        for idx in best_nodes_idx:
            node = map_info.nodes[idx]
            path = map_info.create_path(node)
            if path:
                count += 1
                paths.append(path)
                nodes_new.append(node)
                topo_costs_new.append(topo_costs[idx])
            if count == 5:  # TODO:configurable
                break
        return nodes_new, paths, topo_costs_new

    def weight_costs(self, *cost_vectors, **kwargs):
        costs = self.normalize_costs(numpy.array(tuple(vector for vector in cost_vectors)))
        Print.art_print(str(costs), Print.BLUE)  # TODO:del
        if 'weights' in kwargs:
            weights = kwargs['weights']
        else:
            weights = 2 ** numpy.arange(costs.shape[0] - 1, -1, -1)
        return numpy.average(costs, axis=0, weights=weights)

    # TODO: topo -> topological in file
    def topo_cost(self, node, ogm):
        threshold = self.cost_based_properties['topo_threshold']
        result = 0
        for dir_x, dir_y in [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]:
            cost = 0
            idx = 0
            x, y = node
            while cost < threshold:
                idx += 1
                x_c = x + idx * dir_x
                y_c = y + idx * dir_y
                cost = (x - x_c) ** 2 + (y - y_c) ** 2
                if ogm[x_c][y_c] > 50:
                    break
            result += 1 / min(threshold, cost)
        return result

    @staticmethod
    def distance_cost(path, xy_g):
        weighted_distances = (TargetSelection.distance(node1, node2) * TargetSelection.distance_coeff(node1, xy_g) for
                              node1, node2 in zip(path[:-1], path[1:]))
        return sum(weighted_distances)

    @staticmethod
    def distance_coeff(node, xy_g):
        s = 100
        epsilon = 0.0001
        x_n, y_n = node
        x_g, y_g = xy_g
        coeff = 1 - math.exp(-((x_n - x_g) ** 2 / (2 * s ** 2) + (y_n - y_g) ** 2 / (2 * s ** 2))) + epsilon
        return 1 / coeff

    @staticmethod
    def distance(node_a, node_b):
        x_1, y_1 = node_a
        x_2, y_2 = node_b
        return math.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)

    @staticmethod
    def rotation_cost(path, xy_g, theta):
        rotation = 0
        rx, ry = xy_g
        theta_old = theta
        for node in path:
            st_x, st_y = node
            theta_new = math.atan2(st_y - ry, st_x - rx)
            rotation += abs(theta_new - theta_old)
            theta_old = theta_new
        return rotation

    @staticmethod
    def coverage_cost(path, coverage):
        coverage_sum = sum(coverage[x][y] for x, y in path)
        return -coverage_sum

    @staticmethod
    def normalize_costs(costs):
        """
        :rtype: numpy.ndarray
        """
        Print.art_print("Before normalize:\n" + str(costs), Print.BLUE)  # TODO:del
        # TODO: Should we normalize to [0, 1]?
        return 1 - ((costs.transpose() - costs.min(axis=1)) / costs.ptp(axis=1)).transpose()
