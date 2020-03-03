#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
#import rospy
#from std_msgs.msg import String
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from interval import interval, inf, imath#
from transformations import *

#amankh
import rospy
import ackermann_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
#from reach_flow.msg import FlowstarVisual
import reach_flow.msg
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
import time
 
L = 0.257
L_f = 0.137
L_r = 0.12
counter = 0

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
        self.v_d = 0.7
        self.current_speed = 0
        self.command_speed = 0
        self.command_angle = 0
        self.loop_rate = rospy.Rate(10)
        self.yaw = 0
        self.waypoint_curvature = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.state = [self.pos_x, self.pos_y, self.yaw]
        self.ind = list()
        self.command = [self.command_speed, self.command_angle]
        rospy.Subscriber("ekf_localization/odom", nav_msgs.msg.Odometry, self.stateCallback)
        #rospy.Subscriber("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
        rospy.Subscriber("aa_planner/waypoints", geometry_msgs.msg.Point, self.waypointCallback)
        rospy.Subscriber("aa_planner/commands", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
	#amankh
	self.flow_pub = rospy.Publisher("reach_flow/flow", reach_flow.msg.FlowstarVisual, queue_size=1)
        self.action_pub = rospy.Publisher("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)

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
    def start(self):
	# print(os.getcwd())
        global counter
        #os.remove('flow.txt')
        #os.remove('indicator.txt')
	safety_violated = False # set true if unsafe
        while not rospy.is_shutdown():
            start_time1 = time.time()
            self.state = [self.pos_x, self.pos_y, self.yaw]
            #print 'psi: '+str(self.yaw)
            #print 'current_speed: '+str(self.current_speed)
            #if (self.waypoint_x, self.waypoint_y) == (0.0, 0.0):
            #    print 'current_yaw: '+str(self.waypoint_x)+' '+str(self.waypoint_y)+' '+str(self.yaw)
            #print 'position_x: '+str(self.pos_x)+' / '+str(self.waypoint_x)+ ' / ' +str(self.waypoint_curvature)
            #print 'position_y: '+str(self.pos_y)+' / '+str(self.waypoint_y)
            #print 'angle_command: '+str(self.command_angle)
            #print 'speed_command: '+str(self.command_speed)+'\n'
            #writeFlowstarFile(self.state, self.command, self.waypoint_x, self.waypoint_y, counter)
            flow = executeFlowstar(self.state, self.command, self.waypoint_x, self.waypoint_y, counter)


            counter += 1


            # amankh
            # TO DO correct the flow data
            flow_msg = reach_flow.msg.FlowstarVisual()
            flow_msg.rectangle_x = flow[0]+','+str(self.pos_x)+','+str(self.pos_y)
            flow_msg.rectangle_x2 = flow[1]
            flow_msg.rectangle_y = flow[2]
            flow_msg.rectangle_y2 = flow[3]
	    flow_msg.output_id = int(flow[4])#color, safety indicator
            self.flow_pub.publish(flow_msg)
            print self.command_speed

            action_msg = ackermann_msgs.msg.AckermannDriveStamped()
            if int(flow[4]) == 2 and safety_violated ==False:
                action_msg.drive.speed = self.command_speed
                action_msg.drive.steering_angle = self.command_angle
                self.action_pub.publish(action_msg)
                print 'safe'
            else:
		safety_violated = True
                action_msg.drive.speed = 0.0
                action_msg.drive.steering_angle = 0.0
                self.action_pub.publish(action_msg)
                print 'unsafe'
                #break
            print action_msg.drive.speed, action_msg.drive.steering_angle
            self.loop_rate.sleep()
def executeFlowstar(state, command, waypoint_x, waypoint_y, counter):
    '''
        Function: Execute reachability computation by calling flow*
        Input: none
        Output: none
    '''
    uncertainty = 0.3
    epsilon = 0.1
    pos_x = [state[0]-uncertainty, state[0]+uncertainty]
    pos_y = [state[1]-uncertainty, state[1]+uncertainty]
    yaw = [state[2]-0.01, state[2]+0.01]
    command_speed = [command[0]-0.2, command[0]+0.2]
    command_angle = [command[1]-0.01, command[1]+0.01]
    #os.system('cd model;'+'make clean;'+'make')
    #os.system('pwd')
    mode = 'rounded-circle'
    if mode == 'rounded-circle':
        if (waypoint_x,waypoint_y) == (1, 0):
            formula = str(epsilon)+"+0-y"
            #formula = "10-x"
        if (waypoint_x,waypoint_y) == (2, 1):#x0=1,y0=1
            #formula = "2*x+2*y-x*x-y*y"
            #formula = "10-x"
            #formula = str(epsilon)+"+1-(x-1)*(x-1)+(y-1)*(y-1)"
            #formula = str(epsilon)+"+1-1-1+2*x+2*y-x*x-y*y"
            formula = "-0.9+2*x+2*y-x*x-y*y" 
        if (waypoint_x,waypoint_y) == (2, 2):
            formula = str(epsilon)+"+2-x"
            #formula = "2-x"
            #formula = "10-x"
        if (waypoint_x,waypoint_y) == (1, 3):#x0=1,y0=2
            #formula = str(epsilon)+"+1-(x-1)*(x-1)+(y-2)*(y-2)"
            #formula = "-3+2*x+4*y-x*x-y*y"####error
            #formula = "10-x"
            #formula = str(epsilon)+"+1-1-4+2*x+4*y-x*x-y*y"
            formula = "-3.9+2*x+4*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (0, 3):
            formula = str(epsilon)+"+3-y"
            #formula = "10-x"
        if (waypoint_x,waypoint_y) == (-1, 2):#x0=0,y0=2
            #formula = "-3+4*y-x*x-y*y"########error
            #formula = str(epsilon)+"+1-(x+1)*(x+1)+(y-2)*(y-2)"
            #formula = "10-x"
            #formula = str(epsilon)+"+1-0-4+4*y-x*x-y*y"
            formula = "-2.9+4*y-x*x-y*y"
        if (waypoint_x,waypoint_y) == (-1, 1):
            formula = str(epsilon)+"+x+1"
            #formula = "10-x"
        if (waypoint_x,waypoint_y) == (0, 0):#x0=0, y0=1
            #formula = "2*y-x*x-y*y+1"
            #formula = str(epsilon)+"+1-(x-0)*(x-0)+(y-0)*(y-0)"
            #formula = "10-x"
            formula = "0.5+2*y-x*x-y*y"
    print formula
    os.system('cd model;'+'./RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(command_speed[0]) + ' ' +str(command_speed[1])+ ' '+str(command_angle[0]) + ' '+str(command_angle[1]) + ' '+ formula)
    flow_x_min = list()
    flow_x_max = list()
    flow_y_min = list()
    flow_y_max = list()
    flow_x = list()
    flow_y = list()
    color_list = list()
    with open('model/outputs/RC_bicycle.m', 'r') as f:#one block is a rectange
        for line in f:
            c1 = '['
            c2 = ']'
            #left = [pos for pos, char in enumerate(ll.split('\n')) 
            #block = ll.split('\n')
            if 'plot' in line:
                left = [pos for pos, char in enumerate(line) if char == c1]
                right = [pos for pos, char in enumerate(line) if char == c2]
                string_x = line[left[0]+1:right[0]]
                string_y = line[left[1]+1:right[1]]
                string_color = line[left[2]+1:right[2]]
                x_list = [float(item) for item in string_x.split(',')]
                y_list = [float(item) for item in string_y.split(',')]
                color_line = [float(item) for item in string_color.split(' ')]
                if color_line[0] > 0:
                    color_list.append('red')
                elif color_line[2] > 0:
                    color_list.append('blue')
                else:
                    color_list.append('green')     
                #x_c =  0.5*(min(x_list)+max(x_list))
                #y_c =  0.5*(min(y_list)+max(y_list))
                #width = max(x_list)-min(x_list)
                #height = max(y_list)-min(y_list)
                #flow.append(str(x_c)+','+str(y_c)+','+str(width)+','+str(height))
                flow_x_min.append(str(min(x_list)))
                flow_x_max.append(str(max(x_list)))
                flow_y_min.append(str(min(y_list)))
                flow_y_max.append(str(max(y_list)))
                flow_x.append(string_x)
                flow_y.append(string_y)
                #print x_list
                #print y_list
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
        #print flow_itr
    #return [flow_x_min_itr, flow_x_max_itr, flow_y_min_itr, flow_y_max_itr, color]
    return [flow_x_min_itr, flow_x_max_itr, flow_x_itr, flow_y_itr, color]
def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

def visulization(counter):
    '''
        Function: Visualize reachability results
        Input: none
        Output: none
    '''
    file = 'gnuplot ' + '"RC_bicycle'+str(counter)+'.plt"'
    os.system('cd flowstar-2.1.0/outputs;'+file)
    #os.system('cd flowstar-2.1.0/outputs/images')
    #os.system('mv RC_bicycle.eps RC_bicycle'+str(counter)+'eps')
    
    #imgplot = plt.imshow(mpimg.imread('flowstar-2.1.0/outputs/images/'+'RC_bicycle'+str(counter)+'.eps'))
    #plt.show(block=False)
    #plt.pause(2)
    #plt.close()
def initial_state_interval(delta, uncertainty):
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


