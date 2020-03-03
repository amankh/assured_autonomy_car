#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

Environment for training a local planner to move in a straight line with ROS.
"""

from aa_simulation.envs.straight.straight_env import StraightEnv
from aa_simulation.envs.ros import ros_env_class


class StraightEnvROS(StraightEnv):
    """
    Simulation environment for an RC car following a straight
    line trajectory with ROS.
    """

    HAS_KAPPA = False

    def __init__(self, robot_type = 'RCCar', dt = None, **kwargs):
        """
        Initialize super class, parameters, ROS-related modules.
        """
        ros_class = ros_env_class(robot_type)
        super(StraightEnvROS, self).__init__(robot_type=robot_type, dt=ros_class.DT, **kwargs)

        self._ros = ros_class(self._params)

    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset. Uses our ROS-based
        training framework to pick a new reference frame so that the target
        line is randomly picked w.r.t the vehicle's current position
        """

        state = super(StraightEnvROS,self).get_initial_state

        return self._ros.match_state(state)


    def state_transition(self, action):
        """
        Move one iteration forward by sending action to robot via ROS.
        """
        return self._ros.get_state(action)
