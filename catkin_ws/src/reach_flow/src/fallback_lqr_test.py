#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from interval import interval, inf, imath#
from transformations import *
import rospy
import ackermann_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
#import reach_flow.msg
#import FlowstarVisual.msg
import time
import timeit
import datetime
import matplotlib.pyplot as plt
from uncertainty_estimation_gp import *


def initial_beta_interval(delta_I):
    '''
       Function: compute the interval of beta for the kinematic bicycle model
       Input: steering angle, steering angle uncertainty
    '''
    L = 0.257
    L_f = 0.137
    L_r = 0.12
    #delta_0 = interval[delta-uncertainty, delta+uncertainty]
    beta = (lambda delta: imath.atan(L_r * imath.tan(delta)/L))(delta_I)
    beta_min = min(beta[0][0], beta[0][1])
    beta_max = max(beta[0][0], beta[0][1])
    return [beta_min, beta_max]
if __name__ == '__main__':
    print initial_beta_interval(interval[0.22-0.05, 0.22+0.05])
