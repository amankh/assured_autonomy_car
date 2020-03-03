#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Safety constraint for following a straight line while never driving
away from the line.
"""

import numpy as np

from rllab.core.serializable import Serializable
from sandbox.cpo.safety_constraints.base import SafetyConstraint


class StraightSafetyConstraint(SafetyConstraint, Serializable):
    """
    Always drive towards y=0.
    """

    def __init__(self, max_value=1.0, **kwargs):
        """
        :param max_value: Upper threshold for constraint return
        """
        self.max_value = max_value
        Serializable.quick_init(self, locals())
        super(StraightSafetyConstraint, self).__init__(max_value, **kwargs)


    def evaluate(self, path):
        """
        Return True if constraint is violated.
        """
        observations = path['observations']
        actions = path['actions']
        y = observations[:, 0]
        yaw = observations[:, 1]
        steer = actions[:, 1]

        # Positive if driving away from y=0, negative if driving towards y=0
        driving_away = steer * (np.sign(y) + np.sign(yaw))
        return driving_away > 0

