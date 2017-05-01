#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
from geometry_msgs.msg import Twist

from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation
from sonar_data_aggregator import SonarDataAggregator


# Class for assigning the robot speeds
class RobotController(object):
    MAX_LINEAR_VELOCITY = rospy.get_param('max_linear_velocity')
    MAX_ANGULAR_VELOCITY = rospy.get_param('max_angular_velocity')

    # Constructor
    def __init__(self):

        # Debugging purposes
        self.print_velocities = rospy.get_param('print_velocities')

        # Where and when should you use this?
        self.stop_robot = False

        # Create the needed objects
        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()
        self.navigation = Navigation()

        self.linear_velocity = 0
        self.angular_velocity = 0
        self.stuck_count = 0

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # The timer produces events for sending the speeds every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
        self.velocity_publisher = rospy.Publisher(rospy.get_param('speeds_pub_topic'), Twist, queue_size=10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

        # Produce speeds
        self.produceSpeeds()

        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity

        # Send the command
        self.velocity_publisher.publish(twist)

        # Print the speeds for debuggind purposes
        if self.print_velocities:
            print "[L,R] = [" + str(twist.linear.x) + " , " + str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
        scan = self.laser_aggregation.laser_scan
        ############################### NOTE QUESTION ############################
        # Check what laser_scan contains and create linear and angular speeds
        # for obstacle avoidance
        if not scan:
            return 0, 0

        angle_min = self.laser_aggregation.angle_min
        angle_max = self.laser_aggregation.angle_max

        scan = np.array(scan)
        theta = np.linspace(angle_min, angle_max, len(scan))
        d_virt = 0.2
        scan[scan < d_virt] = d_virt + 0.01
        linear = -sum(np.cos(theta) / ((scan - d_virt) ** 2))
        angular = -sum(np.sin(theta) / ((scan - d_virt) ** 2))
        ##########################################################################
        return linear, angular

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

        # Produce target if not existent
        if self.move_with_target and not self.navigation.target_exists:
            # Create the commands message
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            # Send the command
            self.velocity_publisher.publish(twist)
            self.navigation.selectTarget()

        # Get the submodule's speeds
        l_laser, a_laser = self.produceSpeedsLaser()

        if self.move_with_target:
            ############################### NOTE QUESTION ############################
            # You must combine the two sets of speeds. You can use motor schema,
            # subsumption of whatever suits your better.
            # Initialize robot to goal velocities.
            linear, angular = self.navigation.velocitiesToNextSubtarget()
            c_u = 0.00001
            c_w = 0.00005
            ##########################################################################
        else:
            ############################### NOTE QUESTION ############################
            # Implement obstacle avoidance here using the laser speeds.
            # Hint: Subtract them from something constant
            # Initialize robot to max speed without any change in direction.
            c_u = 0.001
            c_w = 0.005
            angular = 0
            linear = self.MAX_LINEAR_VELOCITY
            ##########################################################################

        angular += c_w * a_laser
        angular = np.sign(angular) * min(self.MAX_ANGULAR_VELOCITY, abs(angular))

        linear = linear + c_u * l_laser
        linear = np.sign(linear) * min(self.MAX_LINEAR_VELOCITY, abs(linear))

        # Heuristic: If the robot is not moving rotate in the direction produced by the laser.
        if self.stuck_count or (
                        abs(linear) + abs(angular) < (self.MAX_LINEAR_VELOCITY + self.MAX_ANGULAR_VELOCITY) / 2 / 6):
            print("Robot was stuck so I set it's speed {}.".format(self.stuck_count))
            linear = self.MAX_LINEAR_VELOCITY * np.sign(l_laser) / 2
            angular = self.MAX_ANGULAR_VELOCITY * np.sign(a_laser)
            self.stuck_count = 0 if self.stuck_count >= 10 else self.stuck_count + 1  # TODO: configurable

        assert abs(angular) <= self.MAX_ANGULAR_VELOCITY, "Angular speed larger than maximum allowed."
        assert abs(linear) <= self.MAX_LINEAR_VELOCITY, "Linear speed larger than maximum allowed."
        self.angular_velocity = angular
        self.linear_velocity = linear

    # Assistive functions
    def stopRobot(self):
        self.stop_robot = True

    def resumeRobot(self):
        self.stop_robot = False
