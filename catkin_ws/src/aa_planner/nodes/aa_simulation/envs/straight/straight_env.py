#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

Environment for training a local planner to move in a straight line.
"""

import numpy as np

from rllab.spaces import Box
from rllab.misc import logger

from aa_simulation.envs.base_env import VehicleEnv
from aa_simulation.misc.utils import normalize_angle


class StraightEnv(VehicleEnv):
    """
    Simulation environment for an RC car following a straight
    line trajectory. The reward function encourages the agent to
    move right on the line y=0 for all time.
    """

    show_distance_history = True

    def __init__(self, lambda1=0.25, **kwargs):
        """
        Initialize super class parameters, obstacles and radius.
        """
        super(StraightEnv, self).__init__(**kwargs)

        # Reward function parameters
        self._lambda1 = lambda1

        # Collect additional data for eval_policy
        self._collected_max_steer_away = 0.0
        self._collected_max_steer_away_y = None
        self._collected_max_steer_away_yaw = None
        self._collected_max_steer_away_cmd = None


    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset.
        """
        # Randomly initialize state for better learning
        if self.robot_type == 'RCCar':
            y = np.random.uniform(-0.25, 0.25)
            yaw = np.random.uniform(-np.pi/3, np.pi/3)
            x_dot = np.random.uniform(0, 1.3)
            y_dot = np.random.uniform(-0.6, 0.6)
            yaw_dot = np.random.uniform(-2.0, 2.0)
        elif self.robot_type == 'MRZR':
            y = np.random.uniform(-0.5, 0.5)
            yaw = np.random.uniform(-np.pi/3, np.pi/3)
            x_dot = np.random.uniform(0, 2*self.target_velocity)
            y_dot = np.random.uniform(-0.6, 0.6)
            yaw_dot = np.random.uniform(-0.3, 0.3)
        else:
            raise ValueError('Unrecognized robot type')

        state = np.zeros(6)
        state[1] = y
        state[2] = yaw
        state[3] = x_dot
        state[4] = y_dot
        state[5] = yaw_dot

        def_action = np.array([self.target_velocity, 0])
        self._prev_actions = np.array([def_action, def_action])

        return state


    def get_reward(self, oldstate, action, state):
        """
        Reward function definition.
        """
        _, y, _, x_dot, y_dot, _ = state
        velocity = np.sqrt(x_dot**2 + y_dot**2)
        distance = y

        reward = -np.absolute(distance)
        reward -= self._lambda1 * (velocity - self.target_velocity)**2

        info = {}
        info['dist'] = distance
        info['vel'] = velocity
        info['jerk'] = action[0] - 2 * self._prev_actions[0][0] + self._prev_actions[1][0]
        info['samount'] = action[1] - self._prev_actions[0][1]
        self._prev_actions[1] = self._prev_actions[0]
        self._prev_actions[0] = action
        return reward, info


    @staticmethod
    def project_line(state, x0, y0, angle):
        """
        Note that this policy is trained to follow a straight line
        to the right (y = 0). To follow an arbitrary line, use this
        function to transform the current absolute state to a form
        that makes the policy believe the car is moving to the right.

        :param state: Current absolute state of robot
        :param x0: x-value of start of line to follow
        :param y0: y-value of start of line to follow
        :param angle: Angle of line to follow
        """
        x, y, yaw, x_dot, y_dot, yaw_dot = state
        angle = normalize_angle(angle)

        current_angle = np.arctan2((y - y0), (x - x0))
        projected_angle = normalize_angle(current_angle - angle)
        dist = np.sqrt((x - x0)**2 + (y - y0)**2)

        new_x = dist * np.cos(projected_angle)
        new_y = dist * np.sin(projected_angle)
        new_yaw = normalize_angle(yaw - angle)

        return np.array([new_x, new_y, new_yaw, x_dot, y_dot, yaw_dot])

    @property
    def observation_min(self):
        return [-np.inf, -np.pi, -np.inf, -np.inf, -np.inf]

    @property
    def observation_max(self):
        return [np.inf, np.pi, np.inf, np.inf, np.inf]

    def state_to_observation(self, state):
        """
        Prepare state to be read as input to neural network.
        """
        _, y, yaw, x_dot, y_dot, yaw_dot = state
        yaw = normalize_angle(yaw)
        return np.array([y, yaw, x_dot, y_dot, yaw_dot])

    def log_diagnostics(self, paths):
        """
        Log extra information per iteration based on collected paths.
        """
        super(StraightEnv, self).log_diagnostics(paths)

        jerks = []
        samounts = []
        for path in paths:
            jerks.append(path['env_infos']['jerk'])
            samounts.append(path['env_infos']['samount'])
        jerks = np.abs(jerks)
        samounts = np.abs(samounts)

        logger.record_tabular('AverageAbsLongitJerk', np.mean(jerks))
        logger.record_tabular('AverageAbsSteerAmount', np.mean(samounts))
        logger.record_tabular('MaxAbsLongitJerk', np.max(jerks))

    def configure_renderer(self, fig, subplot):
        # We import it here, rather than on top of the file to avoid matplotlib
        # dependency in non-rendering use cases
        import matplotlib.pyplot as plt
        super(StraightEnv, self).configure_renderer(fig, subplot)
        plt.axhline(0, color='c', linestyle=':')
        # XXX: TODO: the limits below are for MRZR, should uniformly compute them
        # using self._params instead
        subplot.set_xlim(-2, 200)
        subplot.set_ylim(-50, 50)

    def analyze_rollout(self, path, skip=0):
        super(StraightEnv, self).analyze_rollout(path, skip)
        observations = path['observations']
        actions = path['actions']
        y = observations[:, 0]
        yaw = observations[:, 1]
        steer = actions[:, 1]
        driving_away = steer * (np.sign(y) + np.sign(yaw))
        worst = np.argmax(driving_away)
        if driving_away[worst] > 0:
            scale = abs(y[worst] * yaw[worst] * steer[worst])
            if scale > self._collected_max_steer_away:
                self._collected_max_steer_away = scale
                self._collected_max_steer_away_y = y[worst]
                self._collected_max_steer_away_yaw = yaw[worst]
                self._collected_max_steer_away_cmd = steer[worst]

    def print_means(self):
        super(StraightEnv, self).print_means()
        if self._collected_max_steer_away_y is None:
            print('\tMax Incorrect Steering:\tNone')
        else:
            print('\tMax Incorrect Steering:\t%.5f (y=%.3f, yaw=%.3f -> steer=%.3f)' % (self._collected_max_steer_away, self._collected_max_steer_away_y, self._collected_max_steer_away_yaw, self._collected_max_steer_away_cmd))
