#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

Environment for training local planner to follow circles of
arbitrary curvature with ROS.

----------------------------------------------
Potential TODOs:
    - Read next state after three messages
----------------------------------------------
"""

from aa_simulation.envs.circle.circle_env import CircleEnv
from aa_simulation.envs.ros import ros_env_class


class CircleEnvROS(CircleEnv):
    """
    Simulation environment for an RC car following a circular
    arc trajectory using relative coordinates with ROS.
    """

    HAS_KAPPA = False

    def __init__(self, robot_type, dt=None, **kwargs):
        """
        Initialize super class parameters, obstacles and radius.
        """
        ros_class = ros_env_class(robot_type)
        super(CircleEnvROS, self).__init__(robot_type=robot_type, dt=ros_class.DT, **kwargs)

        self._ros = ros_class(self._params)


    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset.
        """
        state = super(CircleEnvROS,self).get_initial_state
        return self._ros.match_state(state)


    def state_transition(self, action):
        """
        Move one iteration forward by sending action to robot via ROS.
        """
        return self._ros.get_state(action)
