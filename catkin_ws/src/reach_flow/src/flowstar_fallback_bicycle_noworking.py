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
#import FlowstarVisual.msg
import time
import timeit
import datetime
 
L = 0.257
L_f = 0.137
L_r = 0.12
mode = 'circle'#'circle', 'straight',rounded_square

# need to change the os path to reach_flow/src for the file to be called through roslaunch
#-amankh
try:
	os.chdir("/home/nvidia/Documents/FFAST/catkin_ws/src/reach_flow/src")
	print("Working directory changed to reah_flow/src")
except OSError:
	print("Couldn't change the working directory to reach_flow/src. This code will not work with roslaunch or rosrun file")

class Nodo(object):
    def __init__(self, mode):
        global mode
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.command_speed = 0
        self.command_angle = 0
        self.loop_rate = rospy.Rate(10)
        self.yaw = 0
        self.waypoint_curvature = 0
        if mode == 'rounded_square':
            self.waypoint_x = 1
            self.waypoint_y = 0
        else:
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
        self.x_dot = state_t.twist.twist.linear.x
        #self.y_dot = state_t.twist.twist.linear.y

    def actionCallback(self,action_t):
        self.command_speed = action_t.drive.speed
        self.command_angle = action_t.drive.steering_angle

    def waypointCallback(self,waypoint_t):
        self.waypoint_curvature = waypoint_t.z
        self.waypoint_x = waypoint_t.x
        self.waypoint_y = waypoint_t.y
    def start(self):
        total_t = 0
        counter = 0
        max_t = 0
        min_t = 100
        counter_timeout = 0 
	safety_violated = False# set true if unsafe
        #print 'waypoint:', self.waypoint_x,self.waypoint_y
        while not rospy.is_shutdown():
            start = datetime.datetime.now()
            self.state = [self.pos_x, self.pos_y, self.yaw]
            self.command = [self.command_speed, self.command_angle]

	    #####################simulation of malicious command#################
            if np.abs(self.pos_x-1)+np.abs(self.pos_y-0)<=0.1 and (self.waypoint_x,self.waypoint_y) == (2, 1):
                print '################reach position###############'
                print self.state, self.command, self.waypoint_x, self.waypoint_y
                self.command = [2, 0]

            #start = datetime.datetime.now()
            if safety_violated == False:
                flow = executeFlowstar(self.state, self.command, self.waypoint_x, self.waypoint_y)
                last_state = self.state
                last_flow = flow
            #end = datetime.datetime.now()
            #delta_t = (end-start).total_seconds()*1000
            #if delta_t >= 50:
            #    print 'Time /ms: ', delta_t
            #print "after execution:", flow[0]
            
            
            flow_msg = reach_flow.msg.FlowstarVisual()
            flow_msg.initial = str(self.pos_x)+','+str(self.pos_y)
            flow_msg.x = flow[0]
            flow_msg.y = flow[1]
	    flow_msg.indicator = int(flow[2])#safety indicator, 0: unsafe, 1: safe
            
            action_msg = ackermann_msgs.msg.AckermannDriveStamped()
            if flow_msg.indicator == 1 and safety_violated ==False:
                action_msg.drive.speed = self.command_speed
                action_msg.drive.steering_angle = self.command_angle
                self.action_pub.publish(action_msg)
                flow_msg.initial = str(self.pos_x)+','+str(self.pos_y)
                flow_msg.x = flow[0]
                flow_msg.y = flow[1]
                flow_msg.indicator = 1
            else:
		safety_violated = True
                #print self.state
                
                action_msg.drive.speed = 0.0
                action_msg.drive.steering_angle = 0.0
                self.action_pub.publish(action_msg)
                
                flow_msg.initial = str(last_state[0])+','+str(last_state[1])
                flow_msg.x = last_flow[0]
                flow_msg.y = last_flow[1]
                flow_msg.indicator = 0######always display red

                #print 'unsafe', self.state, self.command, self.waypoint_x, self.waypoint_y
                #print "published flow:", flow[0]
                #print flow[1]
                #break
            self.flow_pub.publish(flow_msg)
            end = datetime.datetime.now()
            delta_t = (end-start).total_seconds()*1000
            total_t = total_t+delta_t  
            counter = counter+1
            if delta_t > 100:
                counter_timeout += 1
                print 'timeout percentage %: ', 100*counter_timeout/counter    
            if counter > 0:
                ave_t = total_t/counter
            #print delta_t
            if delta_t >= max_t:
                max_t = delta_t
            if delta_t <= min_t:
                min_t = delta_t
            if counter%1000 == 0:
                print ave_t, max_t, min_t, counter
            self.loop_rate.sleep()
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
    delta_I = interval([min(command[1]-uncertainty_angle, command[1]+uncertainty_angle), max(command[1]-uncertainty_angle, command[1]+uncertainty_angle)])
    #print delta_I
    beta = initial_beta_interval(delta_I)
    
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
        formula = "20-x"
    if mode == 'straight':
        constant = epsilon_line 
        formula = str(constant)+"-y"
        formula = "20-x"

    if mode == 'circle':
        constant = -1-epsilon_cir        
        #formula = str(constant)+"+x*x+y*y"
        formula = "20-x"
 
    co = './RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(beta[0]) + ' ' +str(beta[1])+ ' '+str(command_speed[0]) + ' '+str(command_speed[1]) + ' '+ formula
    #co = './RC_bicycle'+ ' '+ str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(yaw[0]) + ' ' +str(yaw[1])+ ' '+str(command_speed[0]) + ' '+str(command_speed[1]) + ' '+ formula
    #start = datetime.datetime.now()
    os.system('cd model;'+co)
    #print 'co', co
    #end = datetime.datetime.now()
    #delta_t = (end-start).total_seconds()*1000
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
def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode",
            choices=["straight", "circle", "rounded_square"],
            default="rounded_square",
            help="Planner mode")
    args, _ = parser.parse_known_args()
    return args
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
if __name__ == '__main__':
    args = parse_arguments()
    rospy.init_node('reach_flow')
    my_node = Nodo(mode=args.mode)
    my_node.start()
