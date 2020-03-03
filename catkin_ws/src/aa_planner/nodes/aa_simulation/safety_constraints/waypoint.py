#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC and Carnegie Mellon University, 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin, Edward Ahn

Safety constraint for following a waypoint without speeding
"""

import numpy as np

from rllab.core.serializable import Serializable
from sandbox.cpo.safety_constraints.base import SafetyConstraint


class WaypointSafetyConstraint(SafetyConstraint, Serializable):
    """
    Do not go too fast.
    """

    def __init__(self, env, max_speeding_coefficient=1.05, **kwargs):
        """
        :param max_speeding_coefficient: Upper threshold for speeding in proportion to target speed
        :param env: WaypointEnv to get the parameters from
        """
        self.max_value = 1.0
        self._env = env
        self._max_speeding = max_speeding_coefficient
        Serializable.quick_init(self, locals())
        super(WaypointSafetyConstraint, self).__init__(self.max_value, **kwargs)


    def evaluate(self, path):
        """
        Return an array with sum >= 1 iff a constraint is violated.
        """

        # Do not speed
        observations = path['observations']
        actions = path['actions']
        speed = observations[:, 3]
        speed_cmd = actions[:, 0]
        initial_speed = speed[0]
        max_speed = max(self._env._target_speed, initial_speed)
        speed_coeff = speed / (max_speed * self._max_speeding)
        violation = speed_coeff > 1.0
        result = speed_coeff / len(speed_coeff) + violation

        infos = path['env_infos']

        # Do not miss waypoints, with some hefthy weight - want to hit at least 95% of them
        result += np.logical_and(infos['state_type_before'] == 0, infos['state_type'] == -1) * 20.0

        # Do not turn away from waypoints
        y = observations[:, 1]
        yaw = infos['yaw']
        steer = actions[:, 1]
        driving_away = steer * (np.sign(y) + np.sign(yaw))
        # of violations + scale of violation
        result += (driving_away + 1.0 ) * (driving_away > 0)

        return result

