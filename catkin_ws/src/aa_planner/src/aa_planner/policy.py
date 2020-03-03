#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Control the car to run in a trajectory that resembles a square
with rounded corners.
"""

import numpy as np
import os
import six.moves.cPickle as cPickle
import math

from aa_simulation.misc.utils import rotate_state, translate_state, normalize_angle

class Policy(object):
    """
    Run a forward pass of a pre-trained policy.
    """

    def __init__(self, mode, policy_suffix, odom):
        """
        Get model from saved pickle and initialize waypoints.
        """
        self._planner_mode = mode

        # Open saved models.
        dirpath = os.path.dirname(os.path.abspath(__file__))
        if mode == "straight" or mode == "rounded_square":
            with open(dirpath + '/straight_model' + policy_suffix + '.save', 'rb') as f_straight:
                self._straight_model = cPickle.load(f_straight)
        if mode == "circle" or mode == "rounded_square":
            with open(dirpath + '/circle_model' + policy_suffix + '.save', 'rb') as f_circle:
                self._circle_model = cPickle.load(f_circle)
        if mode == "waypoint":
            with open(dirpath + '/waypoint_model' + policy_suffix + '.save', 'rb') as f_waypoint:
                self._waypoint_model = cPickle.load(f_waypoint)
            self._dt = 0.035 # Experimental value?

        # Initialize waypoints to follow rounded square
        self._rounded_square_waypoints = [
                [0,0,np.pi], [1,0,np.pi],
                [2,1,-np.pi/2], [2,2,-np.pi/2],
                [1,3,0], [0,3,0], [-1,2,np.pi/2], [-1,1,np.pi/2]]

        # Parameters for circles to follow
        #   Specified as (center_x, center_y, radius) where
        #   (center_x, center_y) is the center of the circle.
        self._circle_params = [
                [0,1,1], [1,1,1], [1,2,1], [0,2,1]]

        # Parameters for straight lines to follow
        #   Specified as (point_x, point_y, direction) where
        #   (point_x, point_y) is a point on the line and direction
        #   is the direction of the line (0 for y=0 going right,
        #   pi/2 for x=0 going up, -pi/2 for y=0 going left, etc.).
        self._straight_params = [
                [0,0,0], [2,1,np.pi/2],
                [1,3,-np.pi], [-1,2,-np.pi/2]]

        # Initialize meta variables to keep track of current waypoint
        self._on_circle = True
        self._waypoint_num = 0          # 8 looping waypoints
        self._param_num = 0             # 4 params per trajectory type

        # Scale actions (lower and upper bounds)
        # XXX: I [Aleksey Nogin] have no clue where this came from and why it was (is?)
        # necessary. Right now, only used for the straight policy
        self._steer_lb = -np.pi/4
        self._steer_ub = np.pi/4
        self._vel_lb = 0.0
        self._vel_ub = 1.3

        # - A waypoint is a (x,y,yaw,speed) target.
        # - The policy was trained for the car up to 7.5m away from the waypoint
        #   (more specifically, up to 7.5 m in the direction opposite to the heading vector,
        #    and up to 3.75m in either direction perpendicular to the heading vector)
        #   and target speeds of 0.5..3 m/s
        # - In initial state (starting position for the 1st waypoint; previous
        #   waypoint for other waypoints), waypoint should be in a half-plane in front
        #   of the car, rather than the half-plane behind the car
        # - The car is supposed to navigate to each waypoint in turn, then stop
        vx = 4.0
        wp1 = [ (1.0,0.0,0.0,3), (2.0,1.0,np.pi/2,3), (2.0,2.0,np.pi/2,3), (1.0,3.0,-np.pi,3), (0,3,-np.pi,3), (-1,2,-np.pi/2,3), (-1,1,-np.pi/2,3), (0,0,0,3)]
        wp2 = [ (1.0,0.0,0.0,3), (1.5,0.5,np.pi/2,3), (1.0,1.0,-np.pi,3/2), (0.0,1.0,-np.pi,3), (-0.5,0.5,-np.pi/2,3), (0,0,0,1.5),(1.0,0.0,0.0,3), (1.5,0.5,np.pi/2,3), (1.0,1.0,-np.pi,3/2), (0.0,1.0,-np.pi,3), (-0.5,0.5,-np.pi/2,3), (0,0,0,1) ]

        wp3 = [(1.0,0.0,0.0,3), (1.0,1.0,-np.pi,3), (0.0,1.0,-np.pi,3), (0,0,0,3), (1.0,0.0,0.0,3), (1.0,1.0,-np.pi,3), (0.0,1.0,-np.pi,3), (0,0,0,3)]

        wp4 = [ (1.0,0.0,0.0,vx), (1.5,0.5,np.pi/2,vx), (1.0,1.0,-np.pi,vx), (0.0,1.0,-np.pi,vx), (-0.5,0.5,-np.pi/2,vx), (0,0,0,vx) ]

        wp5 = [ (1.0,0.0,0.0,3), (1.3,0.3,np.pi/2,3), (1.0,0.6,-np.pi,3/2), (0.0,0.6,-np.pi,3), (-0.3,0.3,-np.pi/2,3), (0,0,0,1.5),(1.0,0.0,0.0,3), (1.5,0.5,np.pi/2,3), (1.0,1.0,-np.pi,3/2), (0.0,1.0,-np.pi,3), (-0.5,0.5,-np.pi/2,3), (0,0,0,1) ]

        f8 = [(1,0.5,np.pi/2,vx/2), (1.5, 0.5,0,vx), (2,0, -np.pi/2, vx/2), (2.5, -0.5, 0, vx), (3,0,np.pi/2,vx), (2.5, 0.5, -np.pi, vx), (2, 0, -np.pi/2, vx/2), (1.5, -0.5, -np.pi, vx), (1,0,-np.pi,vx), (0,0,-np.pi,1)]
        self._waypoints = wp4
        # bounds on the action, to match how the policies were trained
        self._wp_min_cmd = np.array([0.0, -0.5235987755982988])
        self._wp_max_cmd = np.array([3.0, 0.5235987755982988])

    def get_action(self, state):
        """
        Choose between following a circular, straight, or waypoint-dictated trajectory,
        and use trained models for these trajectory types to
        get an action (desired speed, steering angle) to control
        the car.
        """
        if self._planner_mode == "straight":
            action = self._get_action_straight(state)
            waypoint = [0,0]    # Dummy waypoint
            curvature = 0
            return action, waypoint, curvature

        elif self._planner_mode == "circle":
            action = self._get_action_circle(state)
            waypoint = [0,0]    # Dummy waypoint
            curvature = 0
            return action, waypoint, curvature

        elif self._planner_mode == "rounded_square":
            # Check if state at waypoint and if so, switch trajectory type
            if self._state_at_waypoint(state):
                if not self._on_circle:
                    self._param_num = (self._param_num + 1) % 4
                self._on_circle = not self._on_circle
                self._waypoint_num = (self._waypoint_num + 1) % 8

            if self._on_circle:
                action = self._get_action_circle(state)
            else:
                action = self._get_action_straight(state)

            waypoint = self._rounded_square_waypoints[self._waypoint_num][:2]

            # Odd waypoints are reached via straight lines, even waypoints
            # are reached via circles
            if self._waypoint_num % 2 == 0:
                curvature = 1
            else:
                curvature = 0

            return action, waypoint, curvature

        elif self._planner_mode == "waypoint":
            return self._get_action_waypoint(state)

        else:
            raise ValueError ("Unknown planner mode: " + self._planner_mode)


    def _scaled_action(self, action):
        """
        Scale actions - see rllab's normalized_env's step() function.
        """
        scaled_action = []
        scaled_velocity = self._vel_lb + \
                (action[0] + 1) * 0.5 * (self._vel_ub - self._vel_lb)
        scaled_steering = self._steer_lb + \
                (action[1] + 1) * 0.5 * (self._steer_ub - self._steer_lb)
        scaled_action.append(scaled_velocity)
        scaled_action.append(scaled_steering)
        return scaled_action


    def _state_at_waypoint(self, state):
        """
        Return True if robot state has passed the current waypoint.
        """
        x, y, _, _, _, _ = state
        waypoint = self._rounded_square_waypoints[self._waypoint_num]
        waypoint_x, waypoint_y, waypoint_dir = waypoint

        current_dir = np.arctan2((y - waypoint_y), (x - waypoint_x))
        intersect_angle = normalize_angle(current_dir -
                waypoint_dir)
        return np.absolute(intersect_angle) > np.pi / 2


    def _get_action_circle(self, state):
        """
        Use saved policy to control a car to follow a circular
        trajectory.
        """
        x0, y0, r = self._circle_params[self._param_num]
        x, y, yaw, x_dot, y_dot, yaw_dot = state

        # Translate circle
        x -= x0
        y -= y0

        # Transform state to relative space using convention 2
        dx = np.sqrt(np.square(x) + np.square(y)) - r
        theta = normalize_angle(np.arctan2(-x, y) + np.pi - yaw)
        ddx = x/(x**2 + y**2)**0.5*x_dot + y/(x**2 + y**2)**0.5*y_dot
        dtheta =x/(x**2 + y**2)*x_dot - y/(x**2 + y**2)*y_dot - yaw_dot
        newstate = np.array([dx, theta, ddx, dtheta])

        # Forward pass of policy network
        mean, log_std = [x[0] for x in self._circle_model([newstate])]
        return mean


    def _get_action_straight(self, state):
        """
        Use saved policy to control a car to follow a straight
        trajectory.
        """
        x0, y0, target_dir = self._straight_params[self._param_num]
        x, y, yaw, x_dot, y_dot, yaw_dot = state
        target_dir = target_dir % (2*np.pi)

        # Project line to y=0
        current_dir = np.arctan2((y - y0), (x - x0))
        projection_dir = current_dir - target_dir
        dist = np.sqrt(np.square(x - x0) + np.square(y - y0))
        new_y = dist * np.sin(projection_dir)
        new_yaw = normalize_angle(yaw - target_dir)
        newstate = np.array([new_y, new_yaw, x_dot, y_dot, yaw_dot])

        # Forward pass of policy network
        mean,log_std = [x[0] for x in self._straight_model([newstate])]

        # XXX: It's very unclear to me [Aleksey Nogin] where this scaling came from
        # I removed it from the circle actions (as the policy I trained should be computing
        # correcly scaled outputs already), but left here in case it's actually needed
        return self._scaled_action(mean)


    def _get_action_waypoint(self, state):
        if self._waypoints:
            waypoint = self._waypoints[0]
            speed = math.sqrt(state[3]**2 + state[4]**2)
            # Velocity vector angle relative to the car
            relative_speed_angle = math.atan2(state[4], state[3])
            # Car coordinates in waypoint-centric frame
            relative_state = rotate_state(translate_state(state,(-waypoint[0],-waypoint[1])),-waypoint[2])
            x_dot = speed * math.cos(state[2]+relative_speed_angle) # In waypoint coordinates
            if relative_state[0] + x_dot * 0.5 * self._dt >= 0:
                # We are about to pass the waypoint, focus on the next one
                self._waypoints.pop(0)
                return self._get_action_waypoint(state)
            observation = [
                relative_state[0], #x
                relative_state[1], #y
                normalize_angle(relative_state[2]), # yaw
                speed,
                relative_speed_angle,
                relative_state[5], #yaw_dot
                waypoint[3], #target speed
                speed - waypoint[3], #Speed error
            ]
            mean,log_std = [x[0] for x in self._waypoint_model([observation])]
            action = [
                mean[0]+waypoint[3], # waypoint model computes speed command adjustment relative to target speed
                mean[1],
            ]
            action = np.clip(np.array(action),self._wp_min_cmd, self._wp_max_cmd)
            return (action,waypoint[0:2],waypoint[2])
        else:
            # No waypoints left, stop
            action = [0,0]
            return (action, [0,0], 0)
