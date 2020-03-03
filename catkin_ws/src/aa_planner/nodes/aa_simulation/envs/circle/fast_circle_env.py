#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Environment for training local planner to follow circles as fast as possible.
"""

import numpy as np

from aa_simulation.envs.circle.circle_env import CircleEnv


class FastCircleEnv(CircleEnv):
    """
    Simulation environment for an RC car following a circular
    arc trajectory using relative coordinates as fast as possible.
    """

    def __init__(self, algo='TRPO', eps=0.05, **kwargs):
        """
        Initialize super class parameters, obstacles and radius.
        """
        super(FastCircleEnv, self).__init__(**kwargs)

        self.algo = algo
        self.eps = eps


    def get_reward(self, oldstate, action, state):
        """
        Reward function definition.
        """
        r = self.radius
        x, y, _, x_dot, y_dot, _ = state
        velocity = np.sqrt(x_dot**2 + y_dot**2)
        distance = np.sqrt(x**2 + y**2) - r

        if self.algo == 'TRPO':
            reward = velocity**2
            if distance >= self.eps:
                reward -= 1000000
        elif self.algo == 'CPO':
            reward = velocity**2
        else:
            raise ValueError('Algorithm type unrecognized')

        info = {}
        info['dist'] = distance
        info['vel'] = velocity
        return reward, info

    def configure_renderer(self, fig, subplot):
        # We import it here, rather than on top of the file to avoid matplotlib
        # dependency in non-rendering use cases
        import matplotlib.patches as patches
        # We are intentionally going directly to BaseEnv, rather than CircleEnv
        super(CircleEnv, self).configure_renderer(fig, subplot)
        subplot.set_xlim(-1.5, 1.5)
        subplot.set_ylim(-1.5, 1.5)
        arc = patches.Arc((0, 0), 2-self.eps, 2-self.eps, 0, 0, 360, color='r', ls=':')
        subplot.add_patch(arc)
        arc = patches.Arc((0, 0), 2, 2, 0, 0, 360, color='c')
        subplot.add_patch(arc)
        arc = patches.Arc((0, 0), 2+self.eps, 2+self.eps, 0, 0, 360, color='r', ls=':')
        subplot.add_patch(arc)
