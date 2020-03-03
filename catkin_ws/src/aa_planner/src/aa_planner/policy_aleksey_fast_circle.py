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


class Policy(object):
    """
    Run a forward pass of a pre-trained policy.
    """

    def __init__(self, planner_mode):
        """
        Get model from saved pickle and initialize waypoints.
        """
        self._planner_mode = planner_mode

        # Open saved models.
        dirpath = os.path.dirname(os.path.abspath(__file__))
        f_straight = open(dirpath + '/straight_model.save', 'rb')
        f_circle = open(dirpath + '/circle_model_aleksey.save', 'rb')
        self._straight_model = cPickle.load(f_straight)
        self._circle_model = cPickle.load(f_circle)
        f_straight.close()
        f_circle.close()

        # Initialize waypoints to follow rounded square
        self._waypoints = [
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
        self._steer_lb = -np.pi/4
        self._steer_ub = np.pi/4
        self._vel_lb = 0.0
        self._vel_ub = 1.3


    def get_action(self, state):
        """
        Choose between following a circular or straight trajectory,
        and use trained models for these trajectory types to
        get an action (desired speed, steering angle) to control
        the car.
        """
        if self._planner_mode == "straight":
            action = self._get_action_straight(state)
            waypoint = [0,0]
            curvature = 0
            #return self._scaled_action(action), waypoint, curvature
            return action, waypoint, curvature  # aleksey
        
        elif self._planner_mode == "circle":
            action = self._get_action_circle(state)
            waypoint = [0,0]
            curvature = 0
            return action, waypoint, curvature  # aleksey
#return self._scaled_action(action), waypoint, curvature

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

        waypoint = self._waypoints[self._waypoint_num][:2]

        # Odd waypoints are reached via straight lines, even waypoints
        # are reached via circles
        if self._waypoint_num % 2 == 0:
            curvature = 1
        else:
            curvature = 0

        #return self._scaled_action(action), waypoint, curvature
        return action, waypoint, curvature  # aleksey


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
        waypoint = self._waypoints[self._waypoint_num]
        waypoint_x, waypoint_y, waypoint_dir = waypoint

        current_dir = np.arctan2((y - waypoint_y), (x - waypoint_x))
        intersect_angle = self._normalize_angle(current_dir -
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
        theta = self._normalize_angle(np.arctan2(-x, y) + np.pi - yaw)
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
        new_yaw = self._normalize_angle(yaw - target_dir)
        newstate = np.array([new_y, new_yaw, x_dot, y_dot, yaw_dot])

        # Forward pass of policy network
        mean,log_std = [x[0] for x in self._straight_model([newstate])]
        #return mean
        return self._scaled_action(mean)  # aleksy


    def _normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi).
        """
        angle = angle % (2*np.pi)
        if (angle >= np.pi):
            angle -= 2*np.pi
        return angle
