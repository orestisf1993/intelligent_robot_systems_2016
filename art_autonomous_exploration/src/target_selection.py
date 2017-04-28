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
            pass

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
        # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        found = False
        while not found:
            x_rand = random.randint(0, ogm.shape[0] - 1)
            y_rand = random.randint(0, ogm.shape[1] - 1)
            if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and brushogm[x_rand][y_rand] > 5:
                next_target = [x_rand, y_rand]
                found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), Print.ORANGE)
        return next_target
