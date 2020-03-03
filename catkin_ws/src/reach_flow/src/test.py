#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from interval import interval, inf, imath#
from transformations import *
import numpy as np
import time
import timeit
import datetime
import argparse
 
L = 0.257
L_f = 0.137
L_r = 0.12


def initial_beta_interval(delta, uncertainty):
    '''
       Function: compute the interval of beta for the kinematic bicycle model
       Input: steering angle, steering angle uncertainty
    '''
    global L
    global L_f
    global L_r
    beta = (lambda delta: imath.atan(L_r * imath.tan(delta)/L))(delta_I)
    beta_min = min(beta[0][0], beta[0][1])
    beta_max = max(beta[0][0], beta[0][1])
    return [beta_min, beta_max]
def initial_alpha_f_interval(v_y_I, yaw_dot_I, v_x_I, delta_I):
    '''
       Function: compute the interval of beta for the kinematic bicycle model
       Input: steering angle, steering angle uncertainty
    '''
    global L
    global L_f
    global L_r
    fun = lambda v_y,yaw_dot,v_x,delta: imath.atan((v_y+0.1*yaw_dot)/v_x)-delta
    alpha_f = fun(v_y_I, yaw_dot_I, v_x_I, delta_I)
    return [alpha_f[0][0], alpha_f[0][1]]
#print interval[0.0, 0.1]
#b = interval[0.2, 0.1]
#b1 = interval([0.2, 0.1])
#print b
#print b1
#print list(b)
#a = initial_alpha_f_interval(interval[0.0, 0.1], interval[0.0, 0.1], interval[0.6, 0.61], interval[0.01, 0.02])
#print a
def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode",
            choices=["straight", "circle", "rounded_square"],
            default="rounded_square",
            help="Planner mode")
    args, _ = parser.parse_known_args()
    return args
if __name__ == '__main__':
    args = parse_arguments()
    print args.mode
