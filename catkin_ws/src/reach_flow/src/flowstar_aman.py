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
        self.loop_rate = rospy.Rate(5)
        self.yaw = 0
        self.waypoint_curvature = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.state = [self.pos_x, self.pos_y, self.yaw]
        self.ind = list()
        self.command = [self.command_speed, self.command_angle]
        rospy.Subscriber("ekf_localization/odom", nav_msgs.msg.Odometry, self.stateCallback)
        rospy.Subscriber("commands/keyboard", ackermann_msgs.msg.AckermannDriveStamped, self.actionCallback)
        rospy.Subscriber("aa_planner/waypoints", geometry_msgs.msg.Point, self.waypointCallback)
	#amankh
	self.flow_pub = rospy.Publisher("reach_flow/flow", reach_flow.msg.FlowstarVisual, queue_size=1)

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
        while not rospy.is_shutdown():
            start_time1 = time.time()
            self.state = [self.pos_x, self.pos_y, self.yaw]
            #print 'psi: '+str(self.yaw)
            #print 'current_speed: '+str(self.current_speed)
            #print 'position_x: '+str(self.pos_x)+' / '+str(self.waypoint_x)+ ' / ' +str(self.waypoint_curvature)
            #print 'position_y: '+str(self.pos_y)+' / '+str(self.waypoint_y)
            #print 'angle_command: '+str(self.command_angle)
            #print 'speed_command: '+str(self.command_speed)+'\n'
            #writeFlowstarFile(self.state, self.command, counter)
            flow = executeFlowstar(self.state, self.command, counter)
            #with open('flow.txt', "a") as f:
            #    for i in xrange(len(flow)):
            #        f.write(str(flow[i][0]) + ' ' +str(flow[i][1]) + ' ' + str(flow[i][2]) + ' '+str(flow[i][3]) +'\n')
            #indicator = check_safety(flow)
            #with open('indicator.txt', "a") as h:
            #    h.write(str(counter) + ' ' + str(indicator) + '\n')
            #if indicator == False: #unsafe
            #    print '###################unsafe!!!!!!!!!!!!!#####################'
                #visulization(counter)
            counter += 1


            # amankh
            # TO DO correct the flow data
            flow_msg = reach_flow.msg.FlowstarVisual()
            flow_msg.rectangle_x = flow[0]
            flow_msg.rectangle_x2 = flow[1]
            flow_msg.rectangle_y = flow[2]
            flow_msg.rectangle_y2 = flow[3]
	    flow_msg.output_id = int(counter)
            self.flow_pub.publish(flow_msg)


            self.loop_rate.sleep()

def check_safety(flow):
    indicator = True 
    for i in xrange(len(flow)):
        if flow[i][1]+0.5*flow[i][3] >= 0.05:
            indicator = False
            break
        else:
            continue
    return indicator

def writeFlowstarFile(state, command, counter):
    '''
        Function: Write target point: x_d, y_d, target velocity: v_d, and initial state of auxiliary variable into the template of flow* file
        Input: none
        Output: new flow* file
    '''
    uncertainty = 0.01
    pos_x = state[0]
    pos_y = state[1]
    yaw = state[2]
    command_speed = command[0]
    command_angle = command[1]
    filename = 'model/RC_bicycle_template.cpp'
    with open(filename) as f:
        file_str = f.read()
    #file_str = file_str.replace('RC_bicycle', 'RC_bicycle'+str(counter))
    file_str = file_str.replace('x_max', str(pos_x+uncertainty))
    file_str = file_str.replace('x_min', str(pos_x-uncertainty))
    file_str = file_str.replace('y_max', str(pos_y+uncertainty))
    file_str = file_str.replace('y_min', str(pos_y-uncertainty))
    file_str = file_str.replace('psi_max', str(yaw+uncertainty))
    file_str = file_str.replace('psi_min', str(yaw-uncertainty)) 
    beta = initial_state_interval(command_angle, uncertainty)
    file_str = file_str.replace('beta_max', str(beta[1]))
    file_str = file_str.replace('beta_min', str(beta[0]))
    file_str = file_str.replace('v_max', str(command_speed+uncertainty))
    file_str = file_str.replace('v_min', str(command_speed-uncertainty))
    #with open(filename+'_new'+str(counter)+'.model', "w") as f:
    with open('model/RC_bicycle.cpp', "w") as f:
        f.write(file_str)
def executeFlowstar(state, command, counter):
    '''
        Function: Execute reachability computation by calling flow*
        Input: none
        Output: none
    '''
    uncertainty = 0.1
    pos_x = [state[0]-uncertainty, state[0]+uncertainty]
    pos_y = [state[1]-uncertainty, state[1]+uncertainty]
    yaw = [state[2]-0.01, state[2]+0.01]
    command_speed = [command[0]-0.01, command[0]+0.01]
    command_angle = [command[1]-0.01, command[1]+0.01]
    #os.system('cd model;'+'make clean;'+'make')
    #os.system('pwd')
    os.system('cd model;'+'./RC_bicycle '+str(pos_x[0]) + ' '+ str(pos_x[1]) +' ' + str(pos_y[0]) + ' '+ str(pos_y[1]) + ' ' + str(yaw[0]) +' '+ str(yaw[1]) + ' ' + str(command_speed[0]) + ' ' +str(command_speed[1])+ ' '+str(command_angle[0]) + ' '+str(command_angle[1]))
    flow_x_min = list()
    flow_x_max = list()
    flow_y_min = list()
    flow_y_max = list() 
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
                x_list = [float(item) for item in string_x.split(',')]
                y_list = [float(item) for item in string_y.split(',')]
                #x_c =  0.5*(min(x_list)+max(x_list))
                #y_c =  0.5*(min(y_list)+max(y_list))
                #width = max(x_list)-min(x_list)
                #height = max(y_list)-min(y_list)
                #flow.append(str(x_c)+','+str(y_c)+','+str(width)+','+str(height))
                flow_x_min.append(str(min(x_list)))
                flow_x_max.append(str(max(x_list)))
                flow_y_min.append(str(min(y_list)))
                flow_y_max.append(str(max(y_list))) 
                #print x_list
                #print y_list
        flow_x_min_itr = ','.join(flow_x_min)
        flow_x_max_itr = ','.join(flow_x_max)
        flow_y_min_itr = ','.join(flow_y_min)
        flow_y_max_itr = ','.join(flow_y_max)
        #print flow_itr
    return [flow_x_min_itr, flow_x_max_itr, flow_y_min_itr, flow_y_max_itr]
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


