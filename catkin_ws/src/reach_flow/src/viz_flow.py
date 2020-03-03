#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu


import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from transformations import *

import rospy
import ackermann_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
from reach_flow.msg import FlowstarVisual
import reach_flow.msg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import time
import timeit


mode = 'circle'#'circle', 'straight', 'rounded_square'
# need to change the os path to reach_flow/src for the file to be called through roslaunch
#-amankh
try:
	os.chdir("/home/cfu1/Documents/FFAST/catkin_ws/src/reach_flow/src")
	print("Working directory changed to reah_flow/src")
except OSError:
	print("Couldn't change the working directory to reach_flow/src. This code will not work with roslaunch or rosrun file")

class Nodo(object):
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.yaw = 0
        self.command_speed = 0
        self.command_angle = 0
        self.loop_rate = rospy.Rate(10)
        self.waypoint_curvature = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.indicator = 1
        self.flow_initial = '0,0'#initial position x=0, y=0
        self.flow_x = '-0.1,0.1,0.1,-0.1,-0.1;-0.1,0.1,0.1,-0.1,-0.1'
        self.flow_y = '0.1,0.1,-0.1,-0.1,0.1;0.1,0.1,-0.1,-0.1,0.1'
        self.state = [self.pos_x, self.pos_y, self.yaw]
        self.command = [self.command_speed, self.command_angle]
        rospy.Subscriber("ekf_localization/odom", nav_msgs.msg.Odometry, self.stateCallback)
        rospy.Subscriber("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
        rospy.Subscriber("aa_planner/waypoints", geometry_msgs.msg.Point, self.waypointCallback)
        rospy.Subscriber("reach_flow/flow", reach_flow.msg.FlowstarVisual, self.flowCallback)
        self.loop_rate.sleep()

    def stateCallback(self,state_t):
        self.pos_x = state_t.pose.pose.position.x
        self.pos_y = state_t.pose.pose.position.y
        self.yaw = euler_from_quaternion([self.pos_x,self.pos_y,state_t.pose.pose.orientation.z,state_t.pose.pose.orientation.w])[2]

    def actionCallback(self,action_t):
        self.command_speed = action_t.drive.speed
        self.command_angle = action_t.drive.steering_angle

    def waypointCallback(self,waypoint_t):
        self.waypoint_curvature = waypoint_t.z
        self.waypoint_x = waypoint_t.x
        self.waypoint_y = waypoint_t.y
    def flowCallback(self,flow_t):
        self.flow_initial = flow_t.initial
        self.flow_x = flow_t.x
        self.flow_y = flow_t.y
        self.indicator =  flow_t.indicator
    def start(self,fig):
        global mode
        while not rospy.is_shutdown():
            x_list = []
            y_list = []  
            initial_x = float(self.flow_initial.split(',')[-2])
            initial_y = float(self.flow_initial.split(',')[-1])
            for i in xrange(len(self.flow_x.split(';'))):
                x_temp = self.flow_x.split(';')[i]
                y_temp = self.flow_y.split(';')[i]
                x_list.append(x_temp.split(','))
                y_list.append(y_temp.split(','))#flowpipes for one iteration, e.g., T = 1s, \delta = 0.1, we have 10 boxes
            if self.indicator == 0:
                #print "unsafe ###################################: "
                #print x_list
                #print y_list
                for i in xrange(len(x_list)):
                    plt.plot(x_list[i], y_list[i], 'r')
            if self.indicator == 1:
                for i in xrange(len(x_list)):
                    plt.plot(x_list[i], y_list[i], 'g')
            if mode == "rounded_square":
                plt.plot(initial_x, initial_y, 'r*', markersize=12)#current position

            	plt.plot(np.arange(0, 1.1, 0.1), 0*np.arange(0, 1.1, 0.1), 'b', linewidth = 3.0 )#1st

            	th = np.arange(1.5*np.pi, 2*np.pi+np.pi/10, np.pi/10)
            	plt.plot(1 * np.cos(th)+1, 1 * np.sin(th)+1, 'b', linewidth = 3.0)#2nd

            	plt.plot(2+0*np.arange(1, 2.1, 0.1), np.arange(1, 2.1, 0.1), 'b', linewidth = 3.0)#3rd

            	th = np.arange(0*np.pi, 0.5*np.pi+np.pi/10, np.pi/10)
            	plt.plot(1 * np.cos(th)+1, 1 * np.sin(th)+2, 'b', linewidth = 3.0)#4th

            	plt.plot(np.arange(0, 1.1, 0.1), 3+0*np.arange(0, 1.1, 0.1), 'b', linewidth = 3.0)#5th               

            	th = np.arange(0.5*np.pi, 1.0*np.pi+np.pi/10, np.pi/10)
            	plt.plot(1 * np.cos(th)+0, 1 * np.sin(th)+2, 'b', linewidth = 3.0)#6th

            	plt.plot(-1+0*np.arange(0, 1.1, 0.1), np.arange(1, 2.1, 0.1), 'b', linewidth = 3.0)#7th

            	th = np.arange(np.pi, 1.5*np.pi+np.pi/10, np.pi/10)
            	plt.plot(1 * np.cos(th)+0, 1 * np.sin(th)+1, 'b', linewidth = 3.0)#8th
                plt.xlim(-2, 5)
                plt.ylim(-2, 5)
            if mode == "straight":
                plt.plot(initial_x, initial_y, 'r*', markersize=12)
                plt.plot(np.arange(0, 10, 0.1), 0*np.arange(0, 10, 0.1), 'b', linewidth = 3.0 )
                plt.xlim(-1, 10)
                plt.ylim(-1, 1)
            if mode == "circle":
                plt.plot(initial_x, initial_y, 'r*', markersize=12)
            	th = np.arange(0*np.pi, 2*np.pi+np.pi/10, np.pi/10)
            	plt.plot(1 * np.cos(th), 1 * np.sin(th)+1, 'b', linewidth = 3.0)
                plt.xlim(-2, 2)
                plt.ylim(-1, 3)
            fig.canvas.draw()
            plt.clf()
            self.loop_rate.sleep()
if __name__ == '__main__':
    rospy.init_node('viz_flow')
    my_node = Nodo()
    plt.ion()
    fig = plt.figure(num=None, figsize=(15,15))
    ax = fig.add_subplot(1, 1, 1)
    my_node.start(fig)
