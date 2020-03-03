#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
#import rospy
#from std_msgs.msg import String
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
#from interval import interval, inf, imath#
from transformations import *

#amankh

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
#import matplotlib.image as mpimg
#import time

# need to change the os path to reach_flow/src for the file to be called through roslaunch
#-amankh
try:
	os.chdir("/home/nvidia/Documents/FFAST/catkin_ws/src/reach_flow/src")
	print("Working directory changed to reah_flow/src")
except OSError:
	print("Couldn't change the working directory to reach_flow/src. This code will not work with roslaunch or rosrun file")

historic_size = 10
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
	#amankh
	#self.flow_pub = rospy.Publisher("reach_flow/flow", reach_flow.msg.FlowstarVisual)
        self.loop_rate.sleep()

    def stateCallback(self,state_t):
        self.pos_x = state_t.pose.pose.position.x#same to odom.pose.pose.position.x?
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
    def start(self,fig):
	# print(os.getcwd())
        #global counter
        global xc
        global yc
        global width
        global height
        global historic_size
        #os.remove('flow.txt')
        #os.remove('indicator.txt')
        while not rospy.is_shutdown() and len(self.flow_x_min)>0:
            #print self.flow_x_min
            #start_time1 = time.time()
            self.rec = [self.flow_x_min, self.flow_x_max, self.flow_y_min, self.flow_y_max]
            self.flow_x_min_list = [float(item) for item in self.flow_x_min.split(',')]
            self.flow_x_max_list = [float(item) for item in self.flow_x_max.split(',')]
            self.flow_y_min_list = [float(item) for item in self.flow_y_min.split(',')]
            self.flow_y_max_list = [float(item) for item in self.flow_y_max.split(',')]
 
            self.x_c_i = [0.5*(float(x)+float(y)) for x, y in zip(self.flow_x_max_list, self.flow_x_min_list)]
            self.y_c_i = [0.5*(float(x)+float(y)) for x, y in zip(self.flow_y_max_list, self.flow_y_min_list)]
            self.width_i = [float(x)-float(y) for x, y in zip(self.flow_x_max_list, self.flow_x_min_list)]
            self.height_i = [float(x)-float(y) for x, y in zip(self.flow_y_max_list, self.flow_y_min_list)]


            xc.append(self.x_c_i)
            yc.append(self.y_c_i)
            width.append(self.width_i)
            height.append(self.height_i)
            #with open('flow.txt', "w") as f:
            #    for i in xrange(len(flow)):
            #        f.write(str(flow[i][0]) + ' ' +str(flow[i][1]) + ' ' + str(flow[i][2]) + ' '+str(flow[i][3]) +'\n')
            if len(xc) > historic_size:
                xc1 = xc[-historic_size:]
                yc1 = yc[-historic_size:]
                width1 = width[-historic_size:]
                height1 = height[-historic_size:]
                for i in xrange(len(xc1)):
                    for j in xrange(len(xc1[i])):
                        plt.gca().add_patch(plt.Rectangle((xc1[i][j], yc1[i][j]), width1[i][j], height1[i][j], fill=False, edgecolor='g'))
                plt.xlim(-1, 3)
                plt.ylim(-1, 3)
    		plt.draw()
    		plt.pause(0.001)
    		plt.clf()
            elif len(xc) > 1 and len(xc) <= historic_size:
                xc1 = xc[-1]
                yc1 = yc[-1]
                width1 = width[-1]
                height1 = height[-1]
                #for i in xrange(len(xc1)):
                for j in xrange(len(xc1)):
                    plt.gca().add_patch(plt.Rectangle((xc1[j], yc1[j]), width1[j], height1[j], fill=False, edgecolor='g'))
                plt.xlim(-1, 3)
                plt.ylim(-1, 3)
    		plt.draw()
    		plt.pause(0.001)
    		plt.clf()
            else:
                continue
            #print 'psi: '+str(self.yaw)
            #print 'current_speed: '+str(self.current_speed)
            #print 'position_x: '+str(self.pos_x)+' / '+str(self.waypoint_x)+ ' / ' +str(self.waypoint_curvature)
            #print 'position_y: '+str(self.pos_y)+' / '+str(self.waypoint_y)
            #print 'angle_command: '+str(self.command_angle)
            #print 'speed_command: '+str(self.command_speed)+'\n'
            #writeFlowstarFile(self.state, self.command, counter)
            #flow = executeFlowstar(self.state, self.command, counter)
            #counter += 1
            #print len(xc)
            #if len(xc)>5:
            #    #print len(xc)
            #    ani = animation.FuncAnimation(fig, animate, interval=100)
            #    plt.show()
            #viz(self,fig)
            self.loop_rate.sleep()
def animate(i):
    #xc.append(self.x_c_i)
    #yc.append(self.y_c-i)
    #width.append(self.width_i)
    #height.append(self.height_i)
    global xc
    global yc
    global width
    global height
    global historic_size
    #xc1 = xc[-5:]
    xc1 = xc
    #yc1 = yc[-5:]
    yc1 = yc 
    #width1 = width[-5:]
    width1 = width
    #height1 = height[-5:]
    height1 = height
    
    #print xc1
    #print len(xc1)

        #ax.clear()
    for i in xrange(len(xc1)):
        for j in xrange(len(xc1[i])):
            plt.gca().add_patch(plt.Rectangle((xc1[i][j], yc1[i][j]), width1[i][j], height1[i][j], fill=False, edgecolor='g'))
    ax.set_xlim(-1, 3)
    ax.set_ylim(-1, 3)
    plt.draw()
    plt.pause(0.1)
    plt.clf()
    #return xc
def viz(fig):
    global xc 
    if len(xc) > 5:
        #print len(xc)
        ani = animation.FuncAnimation(fig, animate, interval=100)
        plt.show()
def check_safety(flow):
    indicator = True 
    for i in xrange(len(flow)):
        if flow[i][1]+0.5*flow[i][3] >= 0.05:
            indicator = False
            break
        else:
            continue
    return indicator

def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False
if __name__ == '__main__':
    rospy.init_node('viz_flow')
    my_node = Nodo()
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    #xc = list()
    #yc = list()
    #width = list()
    #height = list()
    my_node.start(fig)
    #viz(fig)
    #ani = animation.FuncAnimation(fig, animate, interval=100)
    #plt.show()
