#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC, 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin

Utility calculations for targeting waypoint & heading. They are kept separate
of the WaypointEnv in order to enable the use of this file in aa_planner
without bringing in all the rllab dependencies with it
"""

import math

class WaypointHeadingCalc(object):
    def __init__(self, max_speed, precision, dt, rmin):
        self._max_v = max_speed
        self._precision = precision
        self._dt = dt
        self._rmin = rmin 

        # Precompute various coefficients used in safety condition
        self._2rmin_sq = (2.0*self._rmin) ** 2

    def set_waypoint(self, curvature = None, min_speed = 0.0, max_speed = None):
        if max_speed is None:
            max_speed = self._max_v
        self._speed_limit = max_speed
        self._2mv_dt = 2.0 * max_speed * self._dt
        self._2mv_dt_mr = self._2mv_dt - self._rmin
        self._sq_2mv_dt = self._2mv_dt ** 2
        self._2rmin_sq_m_sq_2mv_dt = self._2rmin_sq - self._sq_2mv_dt        
        if self._precision < 4.0 * self._max_v * self._dt:
            err = 'Attempted to set precision (%.2f) less than minimal allowed value (4*max_v*dt = %.2f) for the \'%s\' robot type' % (self._precision, 4.0 * self._max_v * self._dt, self.robot_type)
            print(err)
            raise ValueError(err)


    def is_violation(self, x, y, dist, d):
        # XXX: TODO: can/should we allow x to go slightly over 0?
        if x > 0:
            # We passed the waypoint
            return True
        if dist < self._sq_2mv_dt:
            # Center is in the safe zone
            return False
        x_sq = x ** 2
        y_d_rmin = y + d * self._rmin
        if x_sq + y_d_rmin ** 2 < self._2rmin_sq:
            # Cross-distance is too small, corresponding Dubin's path does not exist
            return True
        if d * y > self._2mv_dt_mr:
            # We are far enough on the correct side to not have to worry about not having enough space to turn
            return False
        # are we too far to the side and close to the x=0 line to not have space to turn and got back to y=0
        return self._2rmin_sq_m_sq_2mv_dt * x_sq < (self._2mv_dt * y_d_rmin - self._2rmin_sq) ** 2

    def get_state_type(self, state):
        """
        Returns (state_type, distance, speed, (x_left, y_left, x_right, y_right)), where
          - state_type is 1 when waypoint is reached, -1 when waypoint is
            missed, or vehicle is heading away from the waypoint, and 0
            otherwise
          - (x_left, y_left), (x_right, y_right) are the points r_min to
            the left and right of vehicle respectively.
          - distance is |left - (0,r_min)|**2 + |right - (0,-r_min)|**2
        """

        x, y, yaw, x_dot, y_dot = state[0:5]
        speed = math.sqrt(x_dot**2 + y_dot**2)
        #0.05 is an ad-hoc value
        if speed > 0.05:
            # use velocity direction as heading. x_dot is longitudinal speed, y_dot is lateral speed.
            yaw += math.atan2(y_dot, x_dot)
        c = self._rmin * math.cos(yaw)
        s = self._rmin * math.sin(yaw)
        y_left = y + c
        y_right = y - c
        x_left = x - s
        x_right = x + s
        centers = (x_left, y_left, x_right, y_right)
        distance_left = (y_left-self._rmin)**2 + x_left**2
        distance_right = (y_right+self._rmin)**2 + x_right**2
        distance = math.sqrt(distance_left + distance_right)
        if distance <= self._precision:
            return (1, distance, speed, centers)
        if y_left < y_right \
                            or self.is_violation(x_left, y_left, distance_left, 1) \
                            or self.is_violation(x_right, y_right, distance_right, -1):
            return (-1, distance, speed, centers)
        return (0, distance, speed, centers)
