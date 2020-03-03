#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Jiyuan Zhou

Enable an agent to follow a hard coded trajectory in the form of
a square with rounded corners using trained straight and circle models.
"""

import argparse
import cProfile
import pstats
import sys
import time
import math
import yaml

import joblib
import matplotlib.pyplot as plt
import numpy as np

from rllab.misc import tensor_utils

from aa_simulation.envs.renderer import _Renderer
from aa_simulation.envs.straight_env import StraightEnv


def render(renderer, state, action):
    """
    Render simulation environment.
    """
    renderer.update(state, action)


def modify_state_curve(state, move_param):
    """
    Convert state [x, y, yaw, x_dot, y_dot, yaw_dot] to
        [dx, theta, ddx, dtheta]
    """
    x_0, y_0, r = move_param
    x, y, yaw, x_dot, y_dot, yaw_dot = state

    x -= x_0
    y -= y_0

    dx = np.sqrt(np.square(x) + np.square(y)) - r
    theta = _normalize_angle(np.arctan2(-x, y) + np.pi - yaw)
    ddx = x/(x**2 + y**2)**0.5*x_dot + y/(x**2 + y**2)**0.5*y_dot
    dtheta = x/(x**2 + y**2)*x_dot - y/(x**2 + y**2)*y_dot - yaw_dot

    return np.array([dx, theta, ddx, dtheta])


def _normalize_angle(angle):
    """
    Normalize angle to [-pi, pi).
    """
    angle = angle % (2*np.pi)
    if (angle >= np.pi):
        angle -= 2*np.pi
    return angle


def _normalize_angle2(angle):
    """
    Normalize angle to [0, 2 * pi).
    """
    angle = angle % (2*np.pi)
    return angle


def modify_state_straight(state, move_param):
    """
    Add target direction and target velocity to state, to feed
    in the NN.
    """
    x_0, y_0, target_dir = move_param
    return StraightEnv.project_line(state, x_0, y_0, target_dir)[1:]


def _cal_distance(x, y, move_param):

    init_x, init_y, target_dir = move_param
    position_dir = np.arctan2((y - init_y), (x - init_x))
    projection_dir = _normalize_angle(position_dir - target_dir)

    dist = np.sqrt(np.square(x - init_x) + np.square(y - init_y))

    new_y = dist * np.sin(projection_dir)
    new_x = 0

    return (new_x, new_y)


def _check_point(state, way_point):
    x, y, _, _, _, _ = state
    check_point_x, check_point_y, direction = way_point

    state_direction = np.arctan2((y - check_point_y), (x - check_point_x))

    intersect_angle = _normalize_angle(state_direction - direction)

    return np.absolute(intersect_angle) <= math.pi / 2


def rollout(env, agent, way_point=[], animated=False, speedup=1,
            always_return_paths=False, renderer=None, state=np.zeros(6),
            isCurve=False, move_param=[]):
    observations = []
    actions = []
    rewards = []
    agent_infos = []
    env_infos = []

    path_length = 0

    env._wrapped_env._state = state

    while _check_point(state, way_point):

        if isCurve:
            o = modify_state_curve(state, move_param)
        else:
            o = modify_state_straight(state, move_param)

        _, agent_info = agent.get_action(o)
        a = agent_info['mean']
        next_o, r, d, env_info = env.step(a)

        observations.append(env.observation_space.flatten(o))
        rewards.append(r)
        actions.append(env.action_space.flatten(a))
        agent_infos.append(agent_info)
        env_infos.append(env_info)

        path_length += 1
        if d:
            break

        o = next_o

        state = env._wrapped_env._state

        if animated:
            render(renderer, state, a)
            timestep = 0.0001
            time.sleep(timestep / speedup)
    return state


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speedup', type=float, default=100000,
                        help='Speedup')
    parser.add_argument('--render', dest='render',
            action='store_true', help='Rendering')
    parser.add_argument('--no-render', dest='render',
            action='store_false', help='Rendering')
    parser.set_defaults(render=True)
    args = parser.parse_args()
    return args


def move(env, policy, args, way_point, renderer,\
         state, isCurve, move_param):
    final_state = rollout(env, policy, way_point=way_point,
                        animated=args.render, speedup=args.speedup,
                        always_return_paths=True, renderer=renderer,
                        state=state, isCurve=isCurve,\
                        move_param=move_param)
    return final_state


def init_render():
    stream = open('aa_simulation/envs/model_params.yaml', 'r')
    params = yaml.load(stream)
    return _Renderer(params, None)


def main():
    args = parse_arguments()
    profiler = cProfile.Profile()

    data_curve = joblib.load("data/roundedsquare_demo/circle.pkl")
    policy_curve = data_curve['policy']
    env_curve = data_curve['env']

    data_straight = joblib.load("data/roundedsquare_demo/straight.pkl")
    policy_straight = data_straight['policy']
    env_straight = data_straight['env']

    plt.ion()

    # Set fixed random seed
    np.random.seed(9)

    # Sample one rollout
    profiler.enable()

    # Define initial state
    renderer = init_render()
    state = [0, 0, 0, 0, 0, 0]
    render(renderer, state, None)

    way_points = [
            [0,0,np.pi], [1,0,np.pi],
            [2,1,-np.pi/2], [2,2,-np.pi/2],
            [1,3,0], [0,3,0], [-1,2,np.pi/2], [-1,1,np.pi/2]]
    curve_params = [
            [0,1,1], [1,1,1], [1,2,1], [0,2,1]]
    straight_params = [
            [0,0,0], [2,1,np.pi/2],
            [1,3,-np.pi], [-1,2,-np.pi/2]]

    point = 0
    for i in range(400):

        i %= 4
        # Turn left for 90 degrees
        point %= 8
        state = move(env_curve, policy_curve, args,\
                way_points[point], renderer, state,\
                True, curve_params[i])
        point += 1

        # Move straightly for length 2
        point %= 8
        state = move(env_straight, policy_straight, args,\
                way_points[point], renderer, state,\
                False, straight_params[i])
        point += 1

    profiler.disable()

    # Block until key is pressed
    sys.stdout.write("Press <enter> to continue: ")
    input()


if __name__ == '__main__':
    main()
