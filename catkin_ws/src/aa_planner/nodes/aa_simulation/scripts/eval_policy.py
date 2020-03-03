#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

Evaluate a policy and publish metrics.
"""

import argparse
import cProfile
import pstats
import sys
import os
import json
import re

import joblib
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
import numpy as np

from rllab.sampler.utils import rollout

from aa_simulation.envs.waypoint.waypoint_env import WaypointEnv
from aa_simulation.envs.waypoint.waypoint_env_ros import WaypointEnvROS

from aa_simulation.envs.circle.circle_env import CircleEnv
from aa_simulation.envs.circle.circle_env_ros import CircleEnvROS

from aa_simulation.envs.straight.straight_env import StraightEnv
from aa_simulation.envs.straight.straight_env_ros import StraightEnvROS

def profile_code(profiler):
    """
    Use cProfile to profile code, listing functions with most
    cumulative time spent.
    """
    print('\n')
    stats = pstats.Stats(profiler).strip_dirs()
    stats.sort_stats('cumulative').print_stats(20)
    stats.sort_stats('time').print_stats(20)


def plot_curve(data, name, units):
    """
    Plot data over time.
    """
    if isinstance(data, tuple):
        datas = data
        names = name
    else:
        datas = (data, )
        names = (name, )
    stats = ''
    title = '%s over Time in Final Policy' % (' and '.join(names),)
    plt.figure()
    for i in range(len(datas)):
        data = datas[i]
        name = names[i]
        mean = data.mean()
        std = data.std()
        maximum = data.max()
        minimum = data.min()
        if stats:
            stats += '\n'
        stats += 'Mean = %.5f\nStd = %.5f\nMax = %.5f\nMin = %.5f' % \
                (mean, std, maximum, minimum)
        t = np.arange(data.size)
        plt.plot(t, data)
        plt.axhline(mean, color='k', linestyle='dashed', linewidth=1)
        plt.axhline(mean+std, color='r', linestyle='dashed', linewidth=1)
        plt.axhline(mean-std, color='r', linestyle='dashed', linewidth=1)
    plt.title(title)
    plt.xlabel('Time steps')
    plt.ylabel('%s (%s)' % (name, units))
    plt.text(0.87, 0.98, stats, ha='center', va='top',
            transform=plt.gca().transAxes)

def plot_distribution(data, name, units):
    """
    Plot histogram showing distribution of data.
    """
    if len(data) == 0:
        print ("Warning: '%s' data set is empty, cannot plot distribution!" % (name, ))
        return
    mean = data.mean()
    std = data.std()
    maximum = data.max()
    minimum = data.min()
    stats = 'Mean = %.5f\nStd = %.5f\nMax = %.5f\nMin = %.5f' % \
            (mean, std, maximum, minimum)
    title = 'Distribution of %s in Final Policy' % name

    plt.figure()
    plt.hist(data)
    plt.title(title)
    plt.xlabel('Error (%s)' % units)
    plt.ylabel('Number of Time Steps')
    plt.axvline(mean, color='k', linestyle='dashed', linewidth=1)
    plt.axvline(mean+std, color='r', linestyle='dashed', linewidth=1)
    plt.axvline(mean-std, color='r', linestyle='dashed', linewidth=1)
    plt.text(0.87, 0.9, stats, ha='center', va='center',
            transform=plt.gca().transAxes)


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str,
                        help='Path to the snapshot file')
    parser.add_argument('--max_path_length', type=int, default=None,
                        help='Max length of rollout')
    parser.add_argument('--seed', type=int, default=9, help='Random seed')
    parser.add_argument('--speedup', type=float, default=1,
                        help='Speedup')
    parser.add_argument('--skip', type=int, default=0,
                        help='Number of iterations to skip at start')
    parser.add_argument('--num_paths', type=int, default=1,
                        help='Number of rollouts to collect and evaluate')
    parser.add_argument('--render', dest='render',
            action='store_true', help='Rendering')
    parser.add_argument('--no-render', dest='render',
            action='store_false', help='Rendering')
    parser.set_defaults(render=False)
    parser.add_argument('--plots', dest='show_plots',
            action='store_true', help='Show plots')
    parser.add_argument('--no-plots', dest='show_plots',
            action='store_false', help='Show plots')
    parser.set_defaults(show_plots=True)
    parser.add_argument('--profile', dest='profile_code',
            action='store_true', help='Profile code that samples a rollout')
    parser.add_argument('--no-profile', dest='profile_code',
            action='store_false', help='Profile code that samples a rollout')
    parser.add_argument('--save_movie', type=str, default=None, help='Filename to save a rendering movie to')
    parser.add_argument('--ros', dest='ros', action='store_true',
            help='Use the ROS interface')
    parser.add_argument('--no-ros', dest='ros', action='store_false',
            help='Use the built-in simulator (default)')
    parser.add_argument('--env', choices=['circle', 'straight', 'waypoint'], default='waypoint',
            help='Which environment to use (defaults to waypoint)')
    parser.add_argument('--robot_type', type=str, default='MRZR',
            choices=['MRZR', 'RCCar'])
    parser.add_argument('--variant', type=str, default=None,
            help='Path to variant.json file to use to initialize the experiment (overrides --robot_type)')
    parser.set_defaults(ros=False)
    parser.set_defaults(profile_code=False)
    roslaunch_re = re.compile('^__[a-z]*:=.*$')
    args = parser.parse_args([arg for arg in sys.argv[1:] if not roslaunch_re.match(arg)])
    if args.save_movie is not None and not args.render:
        raise ValueError ("The --no-render option is incompatible with --save_movie")
    return args

class FakePolicy(object):
    def __init__(self, policy):
        self._f_dist = policy

    def reset(self):
        pass

    def get_action(self, observation):
        mean,log_std = [x[0] for x in self._f_dist([observation])]
        return (mean, dict(mean=mean, log_std=log_std))

def main():
    args = parse_arguments()
    profiler = cProfile.Profile()
    data = joblib.load(args.file)
    skip = args.skip

    policy = None
    try:
        policy = data.get('policy', None)
    except:
        pass 
    if policy is None:
        # This is a save file, wrap it around to seem policy-like
        policy=FakePolicy(data)

    if args.variant is not None:
        with open(args.variant, "r") as f:
            variant = json.load(f)
    else:
        variant = dict(target_velocity=4.0, min_speed=0.5, max_speed=6.0, model_type='BrushTireModel', robot_type=args.robot_type)

    if args.save_movie is not None:
        args.save_movie = os.path.abspath(args.save_movie)

    os.chdir(os.path.dirname(sys.argv[0]))
    os.chdir('../..')

    if args.env == 'waypoint':
        env_class = WaypointEnvROS if args.ros else WaypointEnv
    elif args.env == 'straight':
        env_class = StraightEnvROS if args.ros else StraightEnv
    elif args.env == 'circle':
        env_class = CircleEnvROS if args.ros else CircleEnv
    else:
        raise ValueError('Invalid --env argument: ' + args.env)
    env = env_class(**variant)

    np.random.seed(args.seed)

    if args.save_movie is not None:
        FFMpegWriter = manimation.writers['ffmpeg']
        metadata = dict(title=env.robot_type + ' rendering', artist='HRL AA-ExACT Team',
                        comment='Policy file: %s, seed=%i, num_paths=%i'%(args.file, args.seed, args.num_paths))
        writer = (FFMpegWriter(fps=1.0/env._dt, metadata=metadata), args.save_movie)
    else:
        writer = None

    show_plots = args.show_plots
    if show_plots:
        plt.ion()

    env.set_simulation(args.num_paths, writer)

    max_path_length = args.max_path_length
    if max_path_length is None:
        max_path_length = env.horizon

    # rollout uses 0.05 as the default time step for rendering
    speedup = args.speedup * 0.05 / env._dt

    for run in range(args.num_paths):

        if args.profile_code:
            profiler.enable()
        path = rollout(env, policy, max_path_length=max_path_length,
                            animated=args.render, speedup=speedup,
                            always_return_paths=True, use_mean=True)
        if args.profile_code:
            profiler.disable()

        env.analyze_rollout(path, skip)

    if args.profile_code:
        profile_code(profiler)

    if writer:
        writer[0].finish()

    if show_plots:
        # Print statistics over multiple runs
        (curves, distributions) = env.get_plots()
        for (data, title, units) in curves:
            plot_curve(data, title, units)
        for (data, title, units) in distributions:
            plot_distribution(data, title, units)
        plt.show()

    if not args.profile_code:
        print()

    print('Averaged statistics over %d rollout(s):' % args.num_paths)
    env.print_means()
    print()

    if args.render or show_plots:
        # Block until key is pressed
        sys.stdout.write("Press <enter> to continue: ")
        input()


if __name__ == "__main__":
    main()

