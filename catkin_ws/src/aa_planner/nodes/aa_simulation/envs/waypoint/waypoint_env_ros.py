#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin

Environment for training a waypoint controller with ROS.
"""

from aa_simulation.envs.waypoint.waypoint_env import WaypointEnv
from aa_simulation.envs.ros import ros_env_class


class WaypointEnvROS(WaypointEnv):
    """
    Environment for a ROS-based vehicle moving towards a waypoint.
    """

    HAS_KAPPA = False

    def __init__(self, robot_type, dt = None, **kwargs):
        """
        Initialize super class, parameters, ROS-related modules.
        """
        ros_class = ros_env_class(robot_type)
        super(WaypointEnvROS, self).__init__(robot_type=robot_type, dt=ros_class.DT, **kwargs)

        self._ros = ros_class(self._params)

    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset. Uses our ROS-based
        training framework to pick a new reference frame so that the target
        line is randomly picked w.r.t the vehicle's current position
        """

        state_type = 1
        # Since ROS environment's match_state is not exact, we re-check
        # whether we ended up in a state where the waypoint is still feasible
        while state_type != 0:
            state = super(WaypointEnvROS,self).get_initial_state
            state = self._ros.match_state(state)
            state_type, _, _, _ = self._calc.get_state_type(state)

        return state


    def state_transition(self, action):
        """
        Move one iteration forward by sending action to robot via ROS.
        """
        return self._ros.get_state(action)
