#!/usr/bin/env python
from __future__ import division

import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from path_planning import PathPlanning
from robot_perception import RobotPerception
from target_selection import TargetSelection
from utilities import Print
from utilities import RvizHandler


# Class for implementing the navigation module of the robot
class Navigation(object):
    MAX_LINEAR_VELOCITY = rospy.get_param('max_linear_velocity')
    MAX_ANGULAR_VELOCITY = rospy.get_param('max_angular_velocity')

    # Constructor
    def __init__(self):

        # Initializations
        self.robot_perception = RobotPerception()
        self.path_planning = PathPlanning()

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # Flag to check if the vehicle has a target or not
        self.target_exists = False
        self.select_another_target = 0
        self.inner_target_exists = False
        self.force_random = False

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0

        self.count_limit = 200  # 20 sec

        self.counter_to_next_sub = self.count_limit

        # Check if subgoal is reached via a timer callback
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)

        # Read the target function
        self.target_selector = rospy.get_param("target_selector")
        print "The selected target function is " + self.target_selector
        self.target_selection = TargetSelection(self.target_selector)

        # ROS Publisher for the path
        self.path_publisher = rospy.Publisher(rospy.get_param('path_pub_topic'), Path, queue_size=10)

        # ROS Publisher for the subtargets
        self.subtargets_publisher = rospy.Publisher(rospy.get_param('subgoals_pub_topic'), MarkerArray, queue_size=10)

        # ROS Publisher for the current target
        self.current_target_publisher = rospy.Publisher(rospy.get_param('curr_target_pub_topic'), Marker, queue_size=10)

    def checkTarget(self, event):
        # Check if we have a target or if the robot just wanders
        if not self.inner_target_exists or not self.move_with_target or self.next_subtarget == len(self.subtargets):
            return

        self.counter_to_next_sub -= 1
        if self.counter_to_next_sub == 0:
            Print.art_print('\n~~~~ Time reset ~~~~', Print.RED)
            self.inner_target_exists = False
            self.target_exists = False
            self.force_random = True
            return

        # Get the robot pose in pixels
        [rx, ry] = [
            self.robot_perception.robot_pose['x_px'] -
            self.robot_perception.origin['x'] / self.robot_perception.resolution,
            self.robot_perception.robot_pose['y_px'] -
            self.robot_perception.origin['y'] / self.robot_perception.resolution
        ]
        ######################### NOTE: QUESTION  ##############################
        # What if a later subtarget or the end has been reached before the next subtarget? Alter the code
        # accordingly. Check if distance is less than 7 px (14 cm). Slice the list of subtargets and then reverse it.
        for idx, st in enumerate(self.subtargets[self.next_subtarget::][::-1]):
            # Right now, idx refers to the sliced & reversed array, fix it.
            idx = len(self.subtargets) - 1 - idx
            assert idx >= self.next_subtarget
            dist = math.hypot(rx - st[0], ry - st[1])
            if dist < 7:
                self.next_subtarget = idx + 1
                self.counter_to_next_sub = self.count_limit
                if self.next_subtarget == len(self.subtargets):
                    self.target_exists = False
                break
        ########################################################################
        # Publish the current target
        if self.next_subtarget == len(self.subtargets):
            return

        subtarget = [
            self.subtargets[self.next_subtarget][0] * self.robot_perception.resolution +
            self.robot_perception.origin['x'],
            self.subtargets[self.next_subtarget][1] * self.robot_perception.resolution +
            self.robot_perception.origin['y']
        ]
        RvizHandler.printMarker(
            [subtarget],
            1,  # Type: Arrow
            0,  # Action: Add
            "map",  # Frame
            "art_next_subtarget",  # Namespace
            [0, 0, 0.8, 0.8],  # Color RGBA
            0.2  # Scale
        )

    # Function that selects the next target, produces the path and updates
    # the coverage field. This is called from the speeds assignment code, since
    # it contains timer callbacks
    def selectTarget(self):
        # IMPORTANT: The robot must be stopped if you call this function until
        # it is over
        # Check if we have a map
        while not self.robot_perception.have_map:
            Print.art_print("Navigation: No map yet", Print.RED)
            return

        print "\nClearing all markers"
        RvizHandler.printMarker(
            [[0, 0]],
            1,  # Type: Arrow
            3,  # Action: delete all
            "map",  # Frame
            "null",  # Namespace
            [0, 0, 0, 0],  # Color RGBA
            0.1  # Scale
        )

        print '\n\n----------------------------------------------------------'
        print "Navigation: Producing new target"
        # We are good to continue the exploration
        # Make this true in order not to call it again from the speeds assignment
        self.target_exists = True

        # Gets copies of the map and coverage
        local_ogm = self.robot_perception.getMap()
        local_ros_ogm = self.robot_perception.getRosMap()
        local_coverage = self.robot_perception.getCoverage()
        print "Got the map and Coverage"
        self.path_planning.setMap(local_ros_ogm)

        # Once the target has been found, find the path to it
        # Get the global robot pose
        g_robot_pose = self.robot_perception.getGlobalCoordinates(
            [self.robot_perception.robot_pose['x_px'],
             self.robot_perception.robot_pose['y_px']])

        # Call the target selection function to select the next best goal
        # Choose target function
        self.path = []
        while len(self.path) == 0:
            start = time.time()
            target = self.target_selection.selectTarget(
                local_ogm,
                local_coverage,
                self.robot_perception.robot_pose,
                self.robot_perception.origin,
                self.robot_perception.resolution,
                self.force_random)
            self.force_random = False

            self.path = self.path_planning.createPath(
                g_robot_pose,
                target,
                self.robot_perception.resolution)
            print "Navigation: Path for target found with " + str(len(self.path)) + " points"
            if len(self.path) == 0:
                Print.art_print(
                    "Path planning failed. Fallback to random target selection",
                    Print.RED)
                self.force_random = True

        # Reverse the path to start from the robot
        self.path = self.path[::-1]

        # Break the path to subgoals every 2 pixels (1m = 20px)
        step = 1
        n_subgoals = int(len(self.path) / step)
        self.subtargets = []
        for i in range(0, n_subgoals):
            self.subtargets.append(self.path[i * step])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0
        print "The path produced " + str(len(self.subtargets)) + " subgoals"

        ######################### NOTE: QUESTION  ##############################
        # The path is produced by an A* algorithm. This means that it is
        # optimal in length but 1) not smooth and 2) length optimality
        # may not be desired for coverage-based exploration
        ########################################################################

        self.counter_to_next_sub = self.count_limit

        # Publish the path for visualization purposes
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in self.path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ######################### NOTE: QUESTION  ##############################
            # Fill the ps.pose.position values to show the path in RViz
            # You must understand what self.robot_perception.resolution
            # and self.robot_perception.origin are.
            ps.pose.position.x = p[0] * self.robot_perception.resolution + self.robot_perception.origin['x']
            ps.pose.position.y = p[1] * self.robot_perception.resolution + self.robot_perception.origin['y']
            ########################################################################
            ros_path.poses.append(ps)
        self.path_publisher.publish(ros_path)

        # Publish the subtargets for visualization purposes
        subtargets_mark = []
        for s in self.subtargets:
            subt = [
                s[0] * self.robot_perception.resolution + self.robot_perception.origin['x'],
                s[1] * self.robot_perception.resolution + self.robot_perception.origin['y']
            ]
            subtargets_mark.append(subt)

        RvizHandler.printMarker(
            subtargets_mark,
            2,  # Type: Sphere
            0,  # Action: Add
            "map",  # Frame
            "art_subtargets",  # Namespace
            [0, 0.8, 0.0, 0.8],  # Color RGBA
            0.2  # Scale
        )

        self.inner_target_exists = True

    def velocitiesToNextSubtarget(self):
        rx = self.robot_perception.robot_pose['x_px'] - \
             self.robot_perception.origin['x'] / self.robot_perception.resolution
        ry = self.robot_perception.robot_pose['y_px'] - \
             self.robot_perception.origin['y'] / self.robot_perception.resolution
        theta = self.robot_perception.robot_pose['th']
        ######################### NOTE: QUESTION  ##############################
        # The velocities of the robot regarding the next subtarget should be
        # computed. The known parameters are the robot pose [x,y,th] from
        # robot_perception and the next_subtarget [x,y]. From these, you can
        # compute the robot velocities for the vehicle to approach the target.
        # Hint: Trigonometry is required
        if self.subtargets and self.next_subtarget <= len(self.subtargets) - 1:
            st_x = self.subtargets[self.next_subtarget][0]
            st_y = self.subtargets[self.next_subtarget][1]

            theta_g = math.atan2(st_y - ry, st_x - rx)
            delta_theta = theta_g - theta  # All thetas in radians.
            angular_coeff = delta_theta / math.pi + 2 * (delta_theta < -math.pi) - 2 * (delta_theta >= math.pi)
            linear_coeff = (1 - abs(angular_coeff)) ** 6
            # Experimental: Recalculate angular speed according to linear.
            angular_coeff = (1 - linear_coeff) * np.sign(angular_coeff)

            linear = self.MAX_LINEAR_VELOCITY * linear_coeff
            angular = self.MAX_ANGULAR_VELOCITY * angular_coeff
            assert abs(angular_coeff) <= 1, "Angular coefficient outside of range [-1, 1]."
            assert 0 <= linear_coeff <= 1, "Linear coefficient outside of range [0, 1]."
            return linear, angular
        else:
            return 0, 0
