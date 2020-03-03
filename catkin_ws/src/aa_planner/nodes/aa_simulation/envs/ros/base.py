#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

An interface for training a local planner with ROS.
"""

import numpy as np
import rospy
import time
import math
import nav_msgs.msg

from threading import Event, Lock

from aa_simulation.misc.transformations import euler_from_quaternion
from aa_simulation.misc.utils import rotate_state, translate_state

from rllab.core.serializable import Serializable

class ROS_Base(Serializable):
    """
    A generic interface for training a local planner with ROS.
    """

    # Odometry message period, s
    DT = 0.02

    # Number of steps to take when trying to match speed & steering
    _NUM_MATCH_STEPS = 15

    # When matching the desired speed, what fraction has to be matched
    _TARGET_SPEED_FRACTION = 0.9

    # Speed command that is sufficient to get the vehicle moving
    _MIN_SPEED = 0.01

    def __init__(self, params):
        # ROS -> Environment coordinate transformation
        self._rotate = 0
        self._translate = np.zeros(2)

        # Initialize nodes and set up ROS topics
        rospy.init_node('rl_planner')

        self._odom_stamp = rospy.Time.now()
        self._odom_event = Event()

        self._current_state = np.zeros(6)
        self._state_lock = Lock()
        self._last_twist = None
        self._new_position = False

        self.params = params # To keep Serializable happy

        Serializable.quick_init(self, locals())

    def match_state(self, state):
        """
        Try to get the car closer to the given state by:
            - commanding it to a similar speed & steering
            - resetting the coordinate transformation to match the given (x,y,yaw)
        """

        # Reset the coordinate transformation so that we have access to the unmodified
        # ROS state
        with self._state_lock:
            self._rotate = 0
            self._translate = np.zeros(2)

        desired_speed = math.sqrt(state[3]**2 + state[4]**2)
        if desired_speed < self._MIN_SPEED:
            desired_speed = self._MIN_SPEED
        i = 0
        actual_speed = 0

        speed_dir = math.atan2(state[4], state[3])
        desired_steer = speed_dir * 2.0

        # Wait until the command took sufficient effect
        print_threshold = 100
        while i < self._NUM_MATCH_STEPS or actual_speed < self._TARGET_SPEED_FRACTION * desired_speed or desired_speed < actual_speed * self._TARGET_SPEED_FRACTION:
            speed_cmd = desired_speed
            if i > self._NUM_MATCH_STEPS:
                if actual_speed > desired_speed:
                    speed_cmd *= self._TARGET_SPEED_FRACTION
                else:
                    speed_cmd /= self._TARGET_SPEED_FRACTION
            new_state = self.get_state(np.array([speed_cmd, desired_steer]))
            actual_speed = math.sqrt(new_state[3]**2 + new_state[4]**2)
            i += 1
            if (i % print_threshold) == 0:
                print("ROS interface slow to react: set speed %.2fm/s, actual %.2fm/s, %i steps" %(desired_speed,actual_speed,i))
        if i >= print_threshold:
            print("ROS interface was slow to react, now done: set speed %.2fm/s, actual %.2fm/s, %i steps" %(desired_speed,actual_speed,i))

        with self._state_lock:
            # Compute the new coordinate transform and the robot state in the new coordinates
            self._rotate = state[2] - new_state[2]
            new_state = rotate_state(new_state, self._rotate)
            self._translate = np.array([state[0] - new_state[0], state[1] - new_state[1]])
            new_state = translate_state(new_state, self._translate)

        return new_state

    def publish_action(self, action):
        """
        Send the action to robot via ROS.
        """
        raise NotImplementedError("ROS_Base is an abstract class; ROS_Base.publish_action has to be overridden by a subclass")

    def get_state(self, action):
        """
        Publush the action, wait to receive an odometry message, and return
        the vehicle state corresponding to the received odometry message
        """
        num_stuck = 0
        stuck_threshold = 25
        while True:
            self.publish_action(action)
            self._odom_event.clear()
            self._odom_event.wait()
            with self._state_lock:
                if self._new_position:
                    if num_stuck >= stuck_threshold:
                        print("ROS interface: vehicle position started to change")
                    return self._current_state
            num_stuck += 1
            if num_stuck % stuck_threshold == 0:
                print("ROS interface: vehicle position unchanged for %i steps - is Anvel paused? Is WMI in the wrong state?" % (num_stuck,))

    def odometry_callback(self, odom):
        """
        Callback function for odometry state updates.
        """
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = rpy[2]
        x_dot = odom.twist.twist.linear.x
        y_dot = odom.twist.twist.linear.y
        yaw_dot = odom.twist.twist.angular.z
        state = np.array([x, y, yaw, x_dot, y_dot, yaw_dot])
        with self._state_lock:
            self._new_position = (self._last_twist != odom.twist)
            if self._new_position:
                if self._last_twist is None:
                    print("RL Planner ROS interface up")
                self._last_twist = odom.twist
            self._current_state = translate_state(rotate_state(state, self._rotate), self._translate)
            self._odom_stamp = odom.header.stamp
        self._odom_event.set()

    @property
    def odom_stamp(self):
        """
        ROS timestamp of the most recently received odometry message
        """
        with self._state_lock:
            return self._odom_stamp
