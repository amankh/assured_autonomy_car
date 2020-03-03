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
import reach_flow.msg
import time
import timeit
import datetime
 
L = 0.257
L_f = 0.137
L_r = 0.12
mode = 'rounded_square'#'circle', 'straight'
    def start(self):
	safety_violated = False# set true if unsafe
        print 'waypoint:', self.waypoint_x,self.waypoint_y
        while not rospy.is_shutdown():
state = [pos_x, pos_y, yaw]
command = [command_speed, command_angle]
flow = executeFlowstar(state, command, waypoint_x, waypoint_y)
def executeFlowstar(state, command, waypoint_x, waypoint_y):
    '''
        Function: External execution of reachability computation by calling ./RC_bicycle
        Input: state, command, waypoint coordinate
        Output: flowpipe points for visulization, safety indicator
    '''
    uncertainty_pos = 0.1
    uncertainty_angle = 0.01
    uncertainty_speed = 0.05
    epsilon_cir = 0.1 
    epsilon_line = 0.5
    pos_x = [min(state[0]-uncertainty_pos, state[0]+uncertainty_pos), max(state[0]-uncertainty_pos, state[0]+uncertainty_pos)]
    pos_y = [min(state[1]-uncertainty_pos, state[1]+uncertainty_pos), max(state[1]-uncertainty_pos, state[1]+uncertainty_pos)]
    yaw = [min(state[2]-uncertainty_angle, state[2]+uncertainty_angle), max(state[2]-uncertainty_angle, state[2]+uncertainty_angle)]
    command_speed = [min(command[0]-uncertainty_speed, command[0]+uncertainty_speed), max(command[0]-uncertainty_speed, command[0]+uncertainty_speed)]
    delta_I = [min(command[1]-uncertainty_angle, command[1]+uncertainty_angle), max(command[1]-uncertainty_angle, command[1]+uncertainty_angle)]
    beta = initial_beta_interval[delta_I[0], delta_I[1]]
    
    global mode
    if mode == 'rounded_square':
        if (waypoint_x,waypoint_y) == (1, 0):
            constant = epsilon_line 
            formula = str(constant)+"-y"
        if (waypoint_x,waypoint_y) == (2, 1):#x0=1,y0=1
            constant = -1-1+(epsilon_cir+1)*(epsilon_cir+1)
            formula = str(constant)+"+2*x+2*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (2, 2):
            constant = epsilon_line+2
            formula = str(constant)+"-x"
        if (waypoint_x,waypoint_y) == (1, 3):#x0=1,y0=2
            constant = (epsilon_cir+1)*(epsilon_cir+1)-5
            formula = str(constant)+"+2*x+4*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (0, 3):
            constant = epsilon_line+3
            formula = str(constant)+"-y"
        if (waypoint_x,waypoint_y) == (-1, 2):#x0=0,y0=2
            constant = (epsilon_cir+1)*(epsilon_cir+1)-4
            formula = str(constant)+"+4*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (-1, 1):
            constant = epsilon_line+1
            formula = str(constant)+"+x"
        if (waypoint_x,waypoint_y) == (0, 0) and state[2]!=0:#x0=0, y0=1
            constant = (epsilon_cir+1)*(epsilon_cir+1)-1
            formula = str(constant)+"+2*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (0, 0) and abs(state[2])<=0.1:
            constant = epsilon_line 
            formula = str(constant)+"-y"
        #formula = "20-x"
    if mode == 'straight':
        constant = epsilon_line 
        formula = str(constant)+"-y"
        formula = "20-x"

    if mode == 'circle':
        constant = -1-epsilon_cir        
        #formula = str(constant)+"+x*x+y*y"
        formula = "20-x"
 
    co = './RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(beta[0]) + ' ' +str(beta[1])+ ' '+str(command_speed[0]) + ' '+str(command_speed[1]) + ' '+ formula
    start = datetime.datetime.now()
    os.system(co)
    #print 'ttt', ttt
    end = datetime.datetime.now()
    delta_t = (end-start).total_seconds()*1000
    #if delta_t >= 100:
    #    print 'Time /ms: ', delta_t
    flow_x = list()
    flow_y = list()
    color_list = list()
    with open('model/outputs/RC_bicycle.m', 'r') as f:
        for line in f:
            c1 = '['
            c2 = ']'
            if 'plot' in line:
                left = [pos for pos, char in enumerate(line) if char == c1]
                right = [pos for pos, char in enumerate(line) if char == c2]
                string_x = line[left[0]+1:right[0]]
                string_y = line[left[1]+1:right[1]]
                string_color = line[left[2]+1:right[2]]
                x_list = [float(item) for item in string_x.split(',')]#x coordinates every line, for one flowsegment 
                y_list = [float(item) for item in string_y.split(',')]#y coordinates every line, for one flowsegment
                color_line = [float(item) for item in string_color.split(' ')]
                if color_line[0] > 0:
                    color_list.append('red')
                elif color_line[2] > 0:
                    color_list.append('blue')
                else:
                    color_list.append('green')
                flow_x.append(string_x)
                flow_y.append(string_y)
        if 'red' in color_list or 'blue' in color_list:
            indicator = 0#'unsafe'
        else:
            indicator = 1#'safe'
        flow_x_itr = ';'.join(flow_x)#x multiple flowsegments
        flow_y_itr = ';'.join(flow_y)#y multiple flowsegments
    return [flow_x_itr, flow_y_itr, indicator]
def initial_beta_interval(delta_I):
    '''
       Function: compute the interval of beta for the kinematic bicycle model
       Input: steering angle, steering angle uncertainty
    '''
    global L
    global L_f
    global L_r
    #delta_0 = interval[delta-uncertainty, delta+uncertainty]
    beta = (lambda delta: imath.atan(L_r * imath.tan(delta)/L))(delta_I)
    beta_min = min(beta[0][0], beta[0][1])
    beta_max = max(beta[0][0], beta[0][1])
    return [beta_min, beta_max]
#if __name__ == '__main__':
    
