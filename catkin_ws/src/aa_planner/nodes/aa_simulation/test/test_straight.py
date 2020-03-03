#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Test performance of straight line with different projections.
"""

import argparse
import sys

import joblib
import matplotlib.pyplot as plt
import numpy as np

from rllab.misc import tensor_utils

from aa_simulation.envs.straight_env import StraightEnv
from aa_simulation.misc.utils import normalize_angle


def rollout(env, agent, line_params, max_path_length=np.inf,
        animated=False):
    """
    Modified rollout function from rllab.sampler.utils to run
    arbitrary straight trajectories.
    """
    observations = []
    rewards = []
    actions = []
    agent_infos = []
    env_infos = []

    projected_trajectory = []
    x0, y0, angle = line_params
    env.reset()
    agent.reset()

    # Force start state to be zeros
    # Note: Because env is an instance of NormalizedEnv, there is no
    #   way of writing a custom function that I can use to set the
    #   initial state. Consequently we just force set it here.
    start_yaw = angle
    start_state = np.array([x0, y0, start_yaw, 0, 0, 0])
    env._wrapped_env._state = start_state
    o = np.zeros(5)

    path_length = 0
    if animated:
        env.render()
    print('--------------------')
    while path_length < max_path_length:
        print('')
        state = env._wrapped_env._state
        print('State = ', state)
        projected_o = StraightEnv.project_line(state, x0, y0, angle)
        print('Projected state = ', projected_o)
        _, agent_info = agent.get_action(projected_o[1:])
        a = agent_info['mean']
        print('Computed action = ', a)
        next_o, r, d, env_info = env.step(a)
        print('Next observation = ', next_o)
        observations.append(env.observation_space.flatten(o))
        rewards.append(r)
        actions.append(env.action_space.flatten(a))
        agent_infos.append(agent_info)
        env_infos.append(env_info)
        projected_trajectory.append(projected_o)
        path_length += 1
        if d:
            break
        o = next_o
        if animated:
            env.render()
    print('--------------------')

    return dict(
        observations=tensor_utils.stack_tensor_list(observations),
        actions=tensor_utils.stack_tensor_list(actions),
        rewards=tensor_utils.stack_tensor_list(rewards),
        agent_infos=tensor_utils.stack_tensor_dict_list(agent_infos),
        env_infos=tensor_utils.stack_tensor_dict_list(env_infos),
    ), projected_trajectory


def plot_trajectories(trajectory1, trajectory2):
    """
    Plot trajectory of unprojected path and projected path.
    """
    y1 = trajectory1[:,0]
    y2 = trajectory2[:,0]
    t = np.arange(len(y1))

    diff = abs(y2 - y1)
    max_diff = max(diff)
    mean_diff = np.mean(diff)
    print('\nMaximum absolute difference =\t', max_diff)
    print('Mean absolute difference =\t', mean_diff)

    plt.figure()
    plt.title('Trajectories: Relative y-values')
    plt.xlabel('Time step')
    plt.ylabel('y (m)')
    plt.plot(t, y1, 'b', t, y2, 'r')
    plt.legend(['Unprojected', 'Projected'])
    plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str,
                        help='path to the snapshot file')
    parser.add_argument('--max_path_length', type=int, default=100,
                        help='Max length of rollout')
    parser.add_argument('--render', dest='render',
            action='store_true', help='Rendering')
    parser.add_argument('--no-render', dest='render',
            action='store_false', help='Rendering')
    parser.set_defaults(render=False)
    args = parser.parse_args()
    return args


def main():
    args = parse_arguments()
    data = joblib.load(args.file)
    policy = data['policy']
    env = data['env']
    plt.ion()


    #np.set_printoptions(precision=4, suppress=True)


    # Set fixed random seed
    np.random.seed(9)

    # Sample rollouts with different projections (change line_params2)
    line_params1 = np.array([0, 0, 0])
    line_params2 = np.array([3, 0, np.pi/2])
    path1, projected_states1 = rollout(env, policy, line_params1,
            max_path_length=args.max_path_length, animated=args.render)
    path2, projected_states2 = rollout(env, policy, line_params2,
            max_path_length=args.max_path_length, animated=args.render)

    # Plot projected trajectories on graph
    projected_states1 = np.array(projected_states1)
    projected_states2 = np.array(projected_states2)
    plot_trajectories(projected_states1, projected_states2)

    # Block until key is pressed
    sys.stdout.write("Press <enter> to continue: ")
    input()


if __name__ == "__main__":
    main()
