#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

Simulation environment using vehicle model defined in model.py.
"""

import yaml

import math
import numpy as np

from rllab.envs.base import Env, Step
from rllab.misc import logger
from rllab.spaces import Box

from aa_simulation.envs.model.model import BrushTireModel, LinearTireModel
from aa_simulation.envs.renderer import _Renderer


class VehicleEnv(Env):
    """
    Simulation environment for a FFAST RC car.
    """

    # Which eval_policy plots to show
    show_distance_history = False
    speed_plot_name = 'Speed'

    # Model is capable of collecting wheel slip (true for non-ROS environments)
    HAS_KAPPA = True

    def __init__(
            self,
            target_velocity=1.0,
            dt=0.035,
            model_type='BrushTireModel',
            robot_type='RCCar',
            mu_s=1.37,
            mu_k=1.96,
            num_recurrent_dims=0,
            max_recurrent_dim_val=10,
            **_kwargs # Allow passing unused arguments
    ):
        """
        Initialize environment parameters.
        """
        # Load estimated parameters for robot
        if robot_type == 'RCCar':
            stream = open('aa_simulation/envs/model/model_params/rccar.yml','r')
            self._params = yaml.load(stream, Loader=yaml.FullLoader)
        elif robot_type == 'MRZR':
            stream = open('aa_simulation/envs/model/model_params/mrzr.yml','r')
            self._params = yaml.load(stream, Loader=yaml.FullLoader)
        else:
            raise ValueError('Unrecognized robot type')
        self.robot_type = robot_type
        self._min_velocity = 0.0
        self._max_velocity = self._params ['max_v']
        if 'max_steer' in self._params:
            self._max_steer_angle = self._params['max_steer']
        else:
            # Turn radius -> Ackerman steering conversion
            r_min = self._params['r_min']
            wheelbase = (self._params['L_r']+self._params['L_f'])
            self._max_steer_angle = math.atan2(wheelbase, math.sqrt(r_min**2 - self._params['L_r'] **2))
        self._horizon_length = 100

        # Instantiate vehicle model for simulation
        self._state = None
        self._action = None
        self.target_velocity = target_velocity
        if model_type == 'BrushTireModel':
            self._model = BrushTireModel(self._params, mu_s, mu_k)
        elif model_type == 'LinearTireModel':
            self._model = LinearTireModel(self._params, mu_s, mu_k)
        else:
            raise ValueError('Invalid vehicle model type')

        self.num_recurrent_dims = num_recurrent_dims
        self.max_recurrent_dim_val = max_recurrent_dim_val

        self._action_min = [self._min_velocity, -self._max_steer_angle] + [-max_recurrent_dim_val] * num_recurrent_dims
        self._action_max = [self._max_velocity, self._max_steer_angle] + [max_recurrent_dim_val] * num_recurrent_dims

        # Time between each simulation iteration
        # Note: dt is measured to be 0.035, but we train with longer dt
        #       for more stability in commanded actions.
        self._dt = dt

        # Instantiates object handling simulation renderings
        self._renderer = None
        self._simulation_num_paths = None
        self._subplot = None
        self._movie_writer = None

        # Collect data for eval_policy analysis
        self._collected_speeds = []
        self._collected_angles = []
        self._collected_vels = []
        self._collected_means_speed = []
        self._collected_means_steer = []
        self._collected_means_vel = []
        if self.show_distance_history:
            self._collected_dists = []
            self._collected_means_dist = []
        if self.HAS_KAPPA:
            self._collected_kappas = []
            self._collected_means_slip = []

        if _kwargs:
            unused=_kwargs.keys() - [
                # Known training variant attributes not meant for VehicleEnv
                'git status', 'training script', 'git revision',
                '_hidden_keys', 'algo', 'seed', 
                'n_itr', 'num_parallel', 'use_ros', 'pretrained', 'exp_name',
                ]
            if unused:
                print("WARNING: unused VehicleEnv parameters:", ", ".join(unused))

    @property
    def observation_min(self):
        return [-np.inf]*6

    @property
    def observation_max(self):
        return [np.inf]*6

    @property
    def observation_space(self):
        return Box(low=np.array(self.observation_min + [-self.max_recurrent_dim_val] * self.num_recurrent_dims),
                   high=np.array(self.observation_max + [self.max_recurrent_dim_val] * self.num_recurrent_dims))


    @property
    def action_min(self):
        return [self._min_velocity, -self._max_steer_angle]

    @property
    def action_max(self):
        return [self._max_velocity, self._max_steer_angle]

    @property
    def action_space(self):
        return Box(low=np.array(self.action_min + [-self.max_recurrent_dim_val] * self.num_recurrent_dims),
                   high=np.array(self.action_max + [self.max_recurrent_dim_val] * self.num_recurrent_dims))


    @property
    def horizon(self):
        return self._horizon_length


    def reset(self):
        """
        Reset environment back to original state.
        """
        self._action = None
        self._state = self.get_initial_state
        observation = self.state_to_observation(self._state)

        # Reset renderer if available
        if self._renderer is not None:
            self.reset_renderer()

        return observation

    def reset_renderer(self):
        self._renderer.reset()

    def state_transition(self, action):
        """
        Computes the new state resulting from performing the given action on the
        current state
        """
        if self.num_recurrent_dims:
            state = self._state[0:-self.num_recurrent_dims]
        else:
            state = self._state
        return self._model.state_transition(state, action, self._dt)

    def step(self, action):
        """
        Move one iteration forward in simulation.
        """
        # Place limits on action based on mechanical constraints
        action = np.clip(action, a_min=self._action_min, a_max=self._action_max)

        oldstate = self._state
        self._action = action
        nextstate = self.state_transition(action[0:2])
        if self.num_recurrent_dims:
            nextstate = np.append(nextstate, action[2:])
        self._state = nextstate
        reward, info = self.get_reward(oldstate, action, nextstate)
        info['observation'] = self.state_to_observation(nextstate)
        info['reward'] = reward
        if 'done' not in info:
            info['done'] = False
        if self.HAS_KAPPA:
            info['kappa'] = self._model.kappa
        return Step(**info)

    def initialize_renderer(self):
        self._renderer = _Renderer(self)

    def get_render_state(self):
        """
        Compute the state to show in the visualization
        """
        return self._state

    def render(self):
        """
        Render simulation environment.
        """
        if self._renderer == None:
            self.initialize_renderer()
        self._renderer.update(self.get_render_state(), self._action)
        if self._movie_writer:
            self._movie_writer[0].grab_frame()


    def set_simulation(self, num_paths, movie_writer = None):
        """
        Set the environment into simulation (rather than training) mode
        """
        self._simulation_num_paths = num_paths
        self._movie_writer = movie_writer

    def configure_renderer(self, fig, subplot):
        """
        Set the plot dimensions and draw the relevant aspects of the environment
        """
        subplot.set_aspect('equal')
        self._subplot = subplot
        if self._movie_writer:
            writer, filename = self._movie_writer 
            writer.setup(fig, filename, 160) #XXX:TODO:make this DPI value configurable


    def log_diagnostics(self, paths):
        """
        Log extra information per iteration based on collected paths.
        """
        if self.show_distance_history:
            dists = []
        vels = []
        if self.HAS_KAPPA:
            kappas = []
        for path in paths:
            if self.show_distance_history:
                dists.append(path['env_infos']['dist'])
            vels.append(path['env_infos']['vel'])
            if self.HAS_KAPPA:
                kappas.append(path['env_infos']['kappa'])
        if self.show_distance_history:
            dists = np.abs(dists)
        vels = np.abs(vels)
        if self.HAS_KAPPA:
            kappas = np.abs(kappas)

        if self.show_distance_history:
            logger.record_tabular('AverageAbsDistance', np.mean(dists))
            logger.record_tabular('MaxAbsDistance', np.max(dists))
        logger.record_tabular('AverageAbsVelocity', np.mean(vels))
        logger.record_tabular('MaxAbsVelocity', np.max(vels))
        if self.HAS_KAPPA:
            logger.record_tabular('AverageKappa', np.mean(kappas))
            logger.record_tabular('MaxKappa', np.max(kappas))


    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset.
        """
        raise NotImplementedError


    def get_reward(self, oldstate, action, newstate):
        """
        Reward function definition. Returns reward, a scalar, and info, a
        dictionary that must contain the keys 'dist' (closest distance to
        trajectory) and 'vel' (current velocity).
        """
        raise NotImplementedError


    def state_to_observation(self, state):
        """
        Prepare state to be read as input to neural network.
        """
        raise NotImplementedError


    def analyze_rollout(self, path, skip=0):
        """
        Analyze an output of rllab.sampler.utils.rollout for the purposes
        of later displaying statistics from 1 or more rollouts.
        In each rollout, skip the first `skip` steps.
        """
        actions = path['actions']
        self._collected_speeds.append(actions[:, 0][skip:])
        self._collected_angles.append(actions[:, 1][skip:])
        self._collected_vels.append(path['env_infos']['vel'][skip:])

        self._collected_means_speed.append(actions[:, 0][skip:].mean())
        self._collected_means_steer.append(actions[:, 1][skip:].mean())
        self._collected_means_vel.append(path['env_infos']['vel'][skip:].mean())

        if self.HAS_KAPPA:
            self._collected_kappas.append(path['env_infos']['kappa'][skip:])
            self._collected_means_slip.append(path['env_infos']['kappa'][skip:].mean())
        if self.show_distance_history:
            self._collected_dists.append(path['env_infos']['dist'][skip:])
            self._collected_means_dist.append(path['env_infos']['dist'][skip:].mean())

    def get_plots(self):
        """
        From what analyze_rollout had collected, arrange the information to plot
        curves and distributions for
        """
        curves = []
        distributions = []

        speeds = np.concatenate(self._collected_speeds, axis=None)
        #curves.append((speeds, 'Commanded Speed', 'm/s'))
        vels = np.concatenate(self._collected_vels, axis=None)
        #curves.append((vels, self.speed_plot_name, 'm/s'))
        curves.append(((speeds, vels), ("Commanded Speed", self.speed_plot_name), 'm/s'))
        curves.append((np.concatenate(self._collected_angles, axis=None), 'Commanded Steering Angle', 'rad'))
        if self.HAS_KAPPA:
            curves.append((np.concatenate(self._collected_kappas, axis=None), 'Wheel Slip', 'kappa'))
        if self.show_distance_history:
            curves.append((np.concatenate(self._collected_dists, axis=None), 'Distance', 'm'))
        if self.show_distance_history:
            distributions.append((np.concatenate(self._collected_dists, axis=None), 'Distance', 'm'))
        distributions.append((np.concatenate(self._collected_vels, axis=None), self.speed_plot_name, 'm/s'))

        return (curves, distributions)

    def print_means(self):
        """
        From what analyze_rollout had collected, print the summary statistics
        """
        means_speed = np.array(self._collected_means_speed)
        print('\tMean Commanded Speed:\t%.5f +/- %.5f'
            % (means_speed.mean(), means_speed.std()))

        means_steer = np.array(self._collected_means_steer)
        print('\tMean Commanded Steer:\t%.5f +/- %.5f'
            % (means_steer.mean(), means_steer.std()))

        if self.HAS_KAPPA:
            means_slip = np.array(self._collected_means_slip)
            print('\tMean Slip:\t\t%.5f +/- %.5f'
                % (means_slip.mean(), means_slip.std()))

        if self.show_distance_history:
            means_dist = np.array(self._collected_means_dist)
            print('\tMean Distance Error:\t%.5f +/- %.5f'
                % (means_dist.mean(), means_dist.std()))

        means_vel = np.array(self._collected_means_vel)
        print('\tMean %-11s\t%.5f +/- %.5f'
            % (self.speed_plot_name + ":", means_vel.mean(), means_vel.std()))
