#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Test projections with hardcoded actions.
"""

import argparse
import yaml

import joblib
import matplotlib.pyplot as plt
import numpy as np

from aa_simulation.envs.model import VehicleModel
from aa_simulation.envs.straight_env import StraightEnv


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str,
                        help='path to the snapshot file')
    args = parser.parse_args()
    return args


def main():
    args = parse_arguments()
    data = joblib.load(args.file)
    policy = data['policy']
    env = data['env']

    # Set fixed random seed
    np.random.seed(9)

    # Get vehicle model to simulate actions
    stream = open('aa_simulation/envs/model_params.yaml', 'r')
    params = yaml.load(stream)
    model = VehicleModel(params)

    # Experiment parameters
    iterations = 100
    dt = 0.03
    x0 = 2
    y0 = 1
    angle = 0
    state0 = np.zeros(6)
    state1 = np.array([x0, y0, angle, 0, 0, 0])
    diff = []
    t = np.arange(iterations)
    action_hardcode = np.array([0.746, -0.1])

    for i in range(iterations):
        input0 = state0[1:]
        _, action_info0 = policy.get_action(input0)
        action0 = action_info0['mean']
        state0 = model.state_transition(state0, action_hardcode, dt)

        input1 = StraightEnv.project_line(state1, x0, y0, angle)[1:]
        _, action_info1 = policy.get_action(input1)
        action1 = action_info1['mean']
        eps = 1e-5
        state1 = model.state_transition(state1, action_hardcode, dt)
        state1_new = StraightEnv.project_line(state1, x0, y0, angle)

        diff.append(abs(state1_new - state0))

    diff = np.array(diff)

    show_plots = True
    if not show_plots:
        return

    title_type = ['x', 'y', 'yaw', 'x_dot', 'y_dot', 'yaw_dot']
    for i in range(6):
        plt.figure()
        plt.plot(t, diff[:,i])
        title = 'Difference between %s-values'\
                % title_type[i]
        plt.title(title)
        plt.xlabel('Iterations')
        plt.ylabel('Difference')

    plt.show()

if __name__ == "__main__":
    main()
