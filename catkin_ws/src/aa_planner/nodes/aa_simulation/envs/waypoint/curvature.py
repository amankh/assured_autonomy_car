#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC and Carnegie Mellon University, 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin, Stefan Mitsch

Utility calculations for targeting waypoint & curvature. They are kept separate
of the WaypointEnv in order to enable the use of this file in aa_planner
without bringing in all the rllab dependencies with it
"""

import math

# XXX: TODO: enforcing max/min acceleration
class WaypointCurvatureCalc(object):
    def __init__(self, max_speed, precision, dt, rmin):
        self._max_v = max_speed # Only enforced at the goal!!!
        self._precision = precision
        self._dt = dt
        self._rmin = rmin # Not currently enforced!!!

        #TODO!!!
        self._max_acc = 0.01 # Maximal allowed acceleration
        self._max_brk = 0.01 # Maximal allowed braking deceleration

    def set_waypoint(self, curvature, min_speed = 0.0, max_speed = None):
        # These are per waypoint
        self._k = curvature # curvature target
        self._v_low = min_speed
        if max_speed is None:
            self._v_max = self._max_v
        else:
            self._v_max = max_speed

    def get_state_type(self, state, want_print = False):
        """
        Returns (state_type, distance, speed, None), where state_type is 1 when waypoint is
        reached, -1 when waypoint is missed, and 0 otherwise

        """

        x, y, yaw, x_dot, y_dot, _, = state
        speed = math.sqrt(x_dot**2 + y_dot**2)
        #0.05 is an ad-hoc value
        if speed > 0.05:
            # use velocity direction as heading. x_dot is longitudinal speed, y_dot is lateral speed.
            yaw += math.atan2(y_dot, x_dot)
        yg = -x * math.cos(yaw) - y * math.sin(yaw)
        xg =  x * math.sin(yaw) - y * math.cos(yaw)

        distance = math.sqrt(x ** 2 + y ** 2)
        accel = 0 #TODO!!!
        if distance < self._precision and speed <= self._v_max and speed >= self._v_low:
            return (1, distance, speed, None)
        elif (yg>0 and abs(self._k)*self._precision<=1.0 and ((self._k*(self._precision*self._precision)-2*self._precision) < self._k*(xg*xg+yg*yg)-2*xg and self._k*(xg*xg+yg*yg)-2*xg < (self._k*(self._precision*self._precision)+2*self._precision)) and 0<=self._v_low and self._v_low < self._v_max and self._max_acc*self._dt<=(self._v_max-self._v_low) and self._max_brk*self._dt<=(self._v_max-self._v_low)) \
            and ( \
                (-self._max_brk<=accel and accel<=self._max_acc) \
                and \
                speed+accel*self._dt>=0 \
                and \
                ( \
                    (speed<=self._v_max and speed+accel*self._dt<=self._v_max) \
                    or \
                    (1+2*self._precision*abs(self._k)+self._precision*self._precision*(self._k*self._k))*(self._max_brk*(2*speed*self._dt+accel*(self._dt*self._dt))+((speed+accel*self._dt)*(speed+accel*self._dt)-self._v_max*self._v_max))<=2*self._max_brk*(abs(xg)-self._precision) \
                    or \
                    (1+2*self._precision*abs(self._k)+self._precision*self._precision*(self._k*self._k))*(self._max_brk*(2*speed*self._dt+accel*(self._dt*self._dt))+((speed+accel*self._dt)*(speed+accel*self._dt)-self._v_max*self._v_max))<=2*self._max_brk*(yg-self._precision) \
                ) \
                and \
                ( \
                    (self._v_low<=speed and speed+accel*self._dt>=self._v_low) \
                    or \
                    (1+2*self._precision*abs(self._k)+self._precision*self._precision*(self._k*self._k))*(self._max_acc*(2*speed*self._dt+accel*(self._dt*self._dt))+(self._v_low*self._v_low-(speed+accel*self._dt)*(speed+accel*self._dt)))<=2*self._max_acc*(abs(xg)-self._precision) \
                    or \
                    (1+2*self._precision*abs(self._k)+self._precision*self._precision*(self._k*self._k))*(self._max_acc*(2*speed*self._dt+accel*(self._dt*self._dt))+(self._v_low*self._v_low-(speed+accel*self._dt)*(speed+accel*self._dt)))<=2*self._max_acc*(yg-self._precision) \
                )) \
                and \
                (speed>=0 and 0<=self._dt):
            return (0, distance, speed, None)
        else:
            if want_print:
                v="[not computed]"
                print ("violated #%i (x=%.5f,y=%.5f,yaw=%.5f)->(xg=%.5f,yg=%.5f), speed=%.5f, max_speed=%.5f, k=%.5f)" %(v,x,y,yaw,xg,yg,speed,self._v_max,self._k))
 
            return (-1, distance, speed, None)
