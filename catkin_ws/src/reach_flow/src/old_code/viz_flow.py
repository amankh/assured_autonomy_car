#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
#import rospy
#from std_msgs.msg import String
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

# need to change the os path to reach_flow/src for the file to be called through roslaunch
#-amankh
try:
	os.chdir("/home/cfu1/Documents/FFAST/catkin_ws/src/reach_flow/src")
	print("Working directory changed to reah_flow/src")
except OSError:
	print("Couldn't change the working directory to reach_flow/src. This code will not work with roslaunch or rosrun file")

xc = list()
yc = list()
width = list()
height = list()

class Nodo(object):
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.v_d = 0.7
        self.current_speed = 0
        self.command_speed = 0
        self.command_angle = 0
        self.loop_rate = rospy.Rate(10)
        self.yaw = 0
        self.waypoint_curvature = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.indicator = 0
        self.flow_x_min = ''
        self.flow_x_max = ''
        self.flow_y_min = ''
        self.flow_y_max = ''
        self.flow_x_min_list = []
        self.flow_x_max_list = []
        self.flow_y_min_list = []
        self.flow_y_max_list = []
        self.state = [self.pos_x, self.pos_y, self.yaw]
        self.rec = [self.flow_x_min, self.flow_x_max, self.flow_y_min, self.flow_y_max]
        self.x_c_i = list()
        self.y_c_i = list()
        self.width_i = list()
        self.height_i = list()
        self.xc = list()
        self.yc = list()
        self.width = list()
        self.height = list()  
        self.ind = list()
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
        self.x_dot = state_t.twist.twist.linear.x
        self.y_dot = state_t.twist.twist.linear.y
        self.current_speed = np.sqrt(np.square(self.x_dot)+np.square(self.y_dot))

    def actionCallback(self,action_t):
        self.command_speed = action_t.drive.speed
        self.command_angle = action_t.drive.steering_angle

    def waypointCallback(self,waypoint_t):
        self.waypoint_curvature = waypoint_t.z
        self.waypoint_x = waypoint_t.x
        self.waypoint_y = waypoint_t.y
    def flowCallback(self,flow_t):
        self.flow_x_min = flow_t.rectangle_x
        self.flow_x_max = flow_t.rectangle_x2
        self.flow_y_min = flow_t.rectangle_y
        self.flow_y_max = flow_t.rectangle_y2
        self.indicator =  flow_t.output_id
    def start(self,fig):
        #global counter
        global xc
        global yc
        global width
        global height
	counter = 0
        while not rospy.is_shutdown() and len(self.flow_y_min.split(','))>0:
        #while not rospy.is_shutdown():
            #print len(self.flow_x_min)
            start_time1 = time.time()
            self.rec = [self.flow_x_min, self.flow_x_max, self.flow_y_min, self.flow_y_max]
            flow_x_min_list = [float(item) for item in self.flow_x_min.split(',')]
            self.flow_x_min_list = flow_x_min_list[0:10] 
            initial_x = float(self.flow_x_min.split(',')[-2])
            initial_y = float(self.flow_x_min.split(',')[-1])
            self.flow_x_max_list = [float(item) for item in self.flow_x_max.split(',')]
            self.flow_y_min_list = [float(item) for item in self.flow_y_min.split(',')]
            self.flow_y_max_list = [float(item) for item in self.flow_y_max.split(',')]
            
            #flowpipes for one iteration, e.g., T = 1s, \delta = 0.1, we have 10 boxes
            self.x_c_i = [0.5*(float(x)+float(y)) for x, y in zip(self.flow_x_max_list, self.flow_x_min_list)]
            self.y_c_i = [0.5*(float(x)+float(y)) for x, y in zip(self.flow_y_max_list, self.flow_y_min_list)]
            self.width_i = [float(x)-float(y) for x, y in zip(self.flow_x_max_list, self.flow_x_min_list)]
            self.height_i = [float(x)-float(y) for x, y in zip(self.flow_y_max_list, self.flow_y_min_list)]


            xc.append(self.x_c_i)
            yc.append(self.y_c_i)
            width.append(self.width_i)
            height.append(self.height_i)
            his = 2
            if len(xc) > his:
                xc1 = xc[-his:]
                #xc1 = xc
                yc1 = yc[-his:]
                #yc1 = yc
                width1 = width[-his:]
                #width1 = width
                height1 = height[-his:]
                #height1 = height
                #for i in xrange(len(xc1)-1):
                #    for j in xrange(len(xc1[i])):
                #        #plt.gca().add_patch(plt.Rectangle((xc1[i][-1], yc1[i][-1]), width1[i][-1], height1[i][-1], fill=False, edgecolor='grey'))
                #        plt.gca().add_patch(plt.Rectangle((xc1[i][j], yc1[i][j]), width1[i][j], height1[i][j], fill=False, edgecolor='grey'))
                for i in xrange(len(xc1[-1])):
                        if self.indicator == 0:
			    plt.gca().add_patch(plt.Rectangle((xc1[-1][i], yc1[-1][i]), width1[-1][i], height1[-1][i], fill=False, edgecolor='red'))
                        if self.indicator == 1:
                            plt.gca().add_patch(plt.Rectangle((xc1[-1][i], yc1[-1][i]), width1[-1][i], height1[-1][i], fill=False, edgecolor='blue'))
                        if self.indicator == 2:
                            plt.gca().add_patch(plt.Rectangle((xc1[-1][i], yc1[-1][i]), width1[-1][i], height1[-1][i], fill=False, edgecolor='green'))
                if 0 < initial_x < 1 and  2.9 < initial_y < 3.1:
                    print "x-y", initial_x, initial_y
                    print xc1[-1]
                    print yc1[-1]
                    print width1[-1]
                    print height[-1]
                plt.plot(self.pos_x,self.pos_y,'r*',markersize=12)
                plt.plot(initial_x, initial_y, 'b*', markersize=12)
                plt.xlim(-1, 4)
                plt.ylim(-1, 4)
    		fig.canvas.draw()
                #print 'normal_xc1-size', xc1
                #print 'normal_ xc1-i-size', len(xc1[0])
		#plt.draw()
    		#plt.pause(0.01)
    		#plt.clf()
            if len(xc) <= 5 and len(xc)>1:
                xc1 = xc[-1]
                #xc1 = xc
                yc1 = yc[-1]
                #yc1 = yc
                width1 = width[-1]
                #width1 = width
                height1 = height[-1]
                for i in xrange(len(xc1)):
                    plt.gca().add_patch(plt.Rectangle((xc1[i], yc1[i]), width1[i], height1[i], fill=False, edgecolor='g'))
                #plt.gca().add_patch(plt.Rectangle((xc1[-1], yc1[-1]), width1[-1], height1[-1], fill=False, edgecolor='g'))
                plt.plot(self.pos_x,self.pos_y, 'r*',markersize=12)
                plt.plot(initial_x, initial_y, 'b*', markersize=12)
                plt.xlim(-1, 4)
                plt.ylim(-1, 4)
    		fig.canvas.draw()
                #print 'xc1-size', len(xc1)
                #print 'initial-xc1-i-size', len(xc1)
		#plt.draw()
    		#plt.pause(0.01)
    		#plt.clf()
            
            #counter += 1
	    #if counter ==10:
		#fig.clear()
		#plt.clf()
		#counter = 0
            plt.clf()
            #print 'psi: '+str(self.yaw)
            #print 'current_speed: '+str(self.current_speed)
            #print 'position_x: '+str(self.pos_x)+' / '+str(self.waypoint_x)+ ' / ' +str(self.waypoint_curvature)
            #print 'position_y: '+str(self.pos_y)+' / '+str(self.waypoint_y)
            #print 'angle_command: '+str(self.command_angle)
            #print 'speed_command: '+str(self.command_speed)+'\n'
            self.loop_rate.sleep()
if __name__ == '__main__':
    rospy.init_node('viz_flow')
    my_node = Nodo()
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    my_node.start(fig)
