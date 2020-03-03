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
mode = 'rounded-square'#'circle', 'straight'

# need to change the os path to reach_flow/src for the file to be called through roslaunch
#-amankh
try:
	os.chdir("/home/nvidia/Documents/FFAST/catkin_ws/src/reach_flow/src")
	print("Working directory changed to reah_flow/src")
except OSError:
	print("Couldn't change the working directory to reach_flow/src. This code will not work with roslaunch or rosrun file")

class Nodo(object):
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.command_speed = 0
        self.command_angle = 0
        self.loop_rate = rospy.Rate(10)
        self.yaw = 0
        self.waypoint_curvature = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.state = [self.pos_x, self.pos_y, self.yaw]
        self.command = [self.command_speed, self.command_angle]
        rospy.Subscriber("ekf_localization/odom", nav_msgs.msg.Odometry, self.stateCallback)
        #rospy.Subscriber("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
        rospy.Subscriber("aa_planner/waypoints", geometry_msgs.msg.Point, self.waypointCallback)
        rospy.Subscriber("aa_planner/commands", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
	self.flow_pub = rospy.Publisher("reach_flow/flow", reach_flow.msg.FlowstarVisual, queue_size=1)
        self.action_pub = rospy.Publisher("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)

        self.loop_rate.sleep()

    def stateCallback(self,state_t):
        self.pos_x = state_t.pose.pose.position.x#same to odom.pose.pose.position.x?
        self.pos_y = state_t.pose.pose.position.y
        self.yaw = euler_from_quaternion([self.pos_x,self.pos_y,state_t.pose.pose.orientation.z,state_t.pose.pose.orientation.w])[2]
        #self.x_dot = state_t.twist.twist.linear.x
        #self.y_dot = state_t.twist.twist.linear.y

    def actionCallback(self,action_t):
        self.command_speed = action_t.drive.speed
        self.command_angle = action_t.drive.steering_angle

    def waypointCallback(self,waypoint_t):
        self.waypoint_curvature = waypoint_t.z
        self.waypoint_x = waypoint_t.x
        self.waypoint_y = waypoint_t.y
    def start(self):
	safety_violated = False# set true if unsafe
        while not rospy.is_shutdown():
            self.state = [self.pos_x, self.pos_y, self.yaw]
            self.command = [self.command_speed, self.command_angle]

	    #####################simulation of malicious command#################
            if np.abs(self.pos_x-1)+np.abs(self.pos_y-0)<=0.1 and (self.waypoint_x,self.waypoint_y) == (2, 1):
                print '################reach position###############'
                print self.state, self.command, self.waypoint_x, self.waypoint_y
                #self.command = [1.5, 0]

            #start = datetime.datetime.now()
            flow = executeFlowstar(self.state, self.command, self.waypoint_x, self.waypoint_y)
            #end = datetime.datetime.now()
            #delta_t = (end-start).total_seconds()*1000
            #if delta_t >= 50:
            #    print 'Time /ms: ', delta_t

            flow_msg = reach_flow.msg.FlowstarVisual()
            flow_msg.rectangle_x = flow[0]+','+str(self.pos_x)+','+str(self.pos_y)
            flow_msg.rectangle_x2 = flow[1]
            flow_msg.rectangle_y = flow[2]
            flow_msg.rectangle_y2 = flow[3]
	    flow_msg.output_id = int(flow[4])#color, safety indicator

            action_msg = ackermann_msgs.msg.AckermannDriveStamped()
            if int(flow[4]) == 2 and safety_violated ==False:
                action_msg.drive.speed = self.command_speed
                action_msg.drive.steering_angle = self.command_angle
                self.action_pub.publish(action_msg)
            else:
		safety_violated = True
                #print self.state
                action_msg.drive.speed = 0.0
                action_msg.drive.steering_angle = 0.0
                self.action_pub.publish(action_msg)
                flow_msg.output_id = 0######always display red
                print 'unsafe', self.state, self.command, self.waypoint_x, self.waypoint_y
            self.flow_pub.publish(flow_msg)
            self.loop_rate.sleep()
def executeFlowstar(state, command, waypoint_x, waypoint_y):
    '''
        Function: External execution of reachability computation by calling ./RC_bicycle
        Input: state, command, waypoint coordinate
        Output: flowpipe points for visulization, safety indicator
    '''
    uncertainty_pos = 0.1
    uncertainty_angle = 0.01
    uncertainty_speed = 0.2
    epsilon_cir = 0.1
    epsilon_line = 0.5
    pos_x = [min(state[0]-uncertainty_pos, state[0]+uncertainty_pos), max(state[0]-uncertainty_pos, state[0]+uncertainty_pos)]
    pos_y = [min(state[1]-uncertainty_pos, state[1]+uncertainty_pos), max(state[1]-uncertainty_pos, state[1]+uncertainty_pos)]
    yaw = [min(state[2]-uncertainty_angle, state[2]+uncertainty_angle), max(state[2]-uncertainty_angle, state[2]+uncertainty_angle)]
    command_speed = [min(command[0]-uncertainty_speed, command[0]+uncertainty_speed), max(command[0]-uncertainty_speed, command[0]+uncertainty_speed)]
    command_angle = [min(command[1]-uncertainty_angle, command[1]+uncertainty_angle), max(command[1]-uncertainty_angle, command[1]+uncertainty_angle)]
    
    global mode
    if mode == 'rounded-square':
        if (waypoint_x,waypoint_y) == (1, 0):
            constant = epsilon_line 
            formula = str(constant)+"-y"
        if (waypoint_x,waypoint_y) == (2, 1):#x0=1,y0=1
            constant = -1-1+(epsilon_cir+1)*(epsilon_cir+1)
            #constant = -0.79
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
        if (waypoint_x,waypoint_y) == (0, 0):#x0=0, y0=1
            constant = (epsilon_cir+1)*(epsilon_cir+1)-1
            formula = str(constant)+"+2*y-x*x-y*y"
        formula = "20-x"
    if mode == 'straight':
        constant = epsilon_line 
        formula = str(constant)+"-y"
        formula = "20-x"

    if mode == 'circle':
        constant = -1-epsilon_cir        
        #formula = str(constant)+"+x*x+y*y"
        formula = "20-x"
 
    co = './RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(command_angle[0]) + ' ' +str(command_angle[1])+ ' '+str(command_speed[0]) + ' '+str(command_speed[1]) + ' '+ formula
    #print co
    start = datetime.datetime.now()
    os.system('cd model;'+'./RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(command_angle[0]) + ' ' +str(command_angle[1])+ ' '+str(command_speed[0]) + ' '+str(command_speed[1]) + ' '+ formula)
    #print 'ttt', ttt
    end = datetime.datetime.now()
    delta_t = (end-start).total_seconds()*1000
    #if delta_t >= 100:
    #    print 'Time /ms: ', delta_t 
    flow_x_min = list()
    flow_x_max = list()
    flow_y_min = list()
    flow_y_max = list()
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
                x_list = [float(item) for item in string_x.split(',')]#x coordinates very line, multiple flowsegments 
                y_list = [float(item) for item in string_y.split(',')]#y coordinates very line, multiple flowsegments
                color_line = [float(item) for item in string_color.split(' ')]
                if color_line[0] > 0:
                    color_list.append('red')
                elif color_line[2] > 0:
                    color_list.append('blue')
                else:
                    color_list.append('green')     
                flow_x_min.append(str(min(x_list)))
                flow_x_max.append(str(max(x_list)))
                flow_y_min.append(str(min(y_list)))
                flow_y_max.append(str(max(y_list)))
                flow_x.append(string_x)
                flow_y.append(string_y)
        if 'red' in color_list:
            color = 0#'red'
        elif 'blue' in color_list:
            color = 1#'blue'
        else:
            color = 2#'green'
        flow_x_min_itr = ','.join(flow_x_min)
        flow_x_max_itr = ','.join(flow_x_max)
        flow_y_min_itr = ','.join(flow_y_min)
        flow_y_max_itr = ','.join(flow_y_max)
        flow_x_itr = ';'.join(flow_x)
        flow_y_itr = ';'.join(flow_y)
    return [flow_x_min_itr, flow_x_max_itr, flow_x_itr, flow_y_itr, color]
def initial_state_interval(delta, uncertainty):
    '''
       Function: compute the interval of beta for the kinematic bicycle model
       Input: steering angle, steering angle uncertainty
    '''
    global L
    global L_f
    global L_r
    delta_0 = interval[delta-uncertainty, delta+uncertainty]
    beta = (lambda delta: imath.atan(L_r * imath.tan(delta)/L))(delta_0)
    beta_min = min(beta[0][0], beta[0][1])
    beta_max = max(beta[0][0], beta[0][1])
    return [beta_min, beta_max]
if __name__ == '__main__':
    rospy.init_node('reach_flow')
    my_node = Nodo()
    my_node.start()
