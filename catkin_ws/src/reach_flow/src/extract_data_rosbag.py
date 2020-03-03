#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu


import rosbag
from transformations import *
from scipy import signal
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 18})
bag = rosbag.Bag('rounded_square.bag')
print bag.get_type_and_topic_info()

bag_start = bag.get_start_time()
bag_end = bag.get_end_time()

cmd_vel, cmd_steering_angle, cmd_t = list(), list(), list() 
for topic, msg, t in bag.read_messages(topics=['/commands/keyboard']):
    #cmd_vel.append(msg.drive.speed)
    #cmd_steering_angle.append(msg.drive.steering_angle)
    cmd_t.append(t)
    #print msg


pos_x, pos_y, yaw, pos_t = list(), list(), list(), list() 


for topic, msg, t in bag.read_messages(topics=['/ekf_localization/odom']):
    #pos_x.append(msg.pose.pose.position.x)
    #pos_y.append(msg.pose.pose.position.y)
    pos_t.append(t)

waypoints_x, waypoints_y, waypoints_curvature, waypoints_t = list(), list(), list(), list()
for topic, msg, t in bag.read_messages(topics=['/aa_planner/waypoints']):#/aa_planner/waypoints
    waypoints_t.append(t)
    #print msg

cut_start_time = max(cmd_t[0], pos_t[0], waypoints_t[0])
cut_end_time = min(cmd_t[-1], pos_t[-1], waypoints_t[-1])


for topic, msg, t in bag.read_messages(topics=['/ekf_localization/odom'], start_time = cut_start_time, end_time = cut_end_time):
    pos_x.append(msg.pose.pose.position.x)
    pos_y.append(msg.pose.pose.position.y)
    pos_z = msg.pose.pose.orientation.z
    pos_w = msg.pose.pose.orientation.w
    yaw.append(euler_from_quaternion([pos_x[-1], pos_y[-1], pos_z, pos_w])[2])

for topic, msg, t in bag.read_messages(topics=['/aa_planner/waypoints'], start_time = cut_start_time, end_time = cut_end_time):
    waypoints_x.append(msg.x)
    waypoints_y.append(msg.y)
    waypoints_curvature.append(msg.z)

for topic, msg, t in bag.read_messages(topics=['/commands/keyboard'], start_time = cut_start_time, end_time = cut_end_time):
    cmd_vel.append(msg.drive.speed)
    cmd_steering_angle.append(msg.drive.steering_angle)

print len(pos_x), len(pos_y), len(yaw), len(waypoints_x), len(waypoints_y), len(waypoints_curvature), len(cmd_vel), len(cmd_steering_angle)
pos_x_resample = signal.resample(pos_x, len(cmd_vel))
pos_y_resample = signal.resample(pos_y, len(cmd_vel))
yaw_resample = signal.resample(yaw, len(cmd_vel))
waypoints_x_resample = signal.resample(waypoints_x, len(cmd_vel))
waypoints_y_resample = signal.resample(waypoints_y, len(cmd_vel))
waypoints_curvature_resample = signal.resample(waypoints_curvature, len(cmd_vel))
duration_sec = (cut_end_time - cut_start_time).to_sec()
frequency = len(cmd_vel)/duration_sec#39.79 hz



#print frequency
pos_x_10 = signal.resample(pos_x_resample, len(cmd_vel)/4)
pos_y_10 = signal.resample(pos_y_resample, len(cmd_vel)/4)
yaw_10 = signal.resample(yaw_resample, len(cmd_vel)/4)
waypoints_x_10 = signal.resample(waypoints_x_resample, len(cmd_vel)/4)
waypoints_y_10 = signal.resample(waypoints_y_resample, len(cmd_vel)/4)
waypoints_curvature_10 = signal.resample(waypoints_curvature, len(cmd_vel)/4)
cmd_vel_10 = signal.resample(cmd_vel, len(cmd_vel)/4)
cmd_steering_angle_10 = signal.resample(cmd_steering_angle, len(cmd_vel)/4)
print len(pos_x_10), len(pos_y_10), len(yaw_10), len(waypoints_x_10), len(waypoints_y_10)


#diff_cmd_vel, diff_cmd_steering_angle = list(), list()
#for i in xrange(1, len(cmd_vel)):
#    diff_cmd_vel.append(cmd_vel[i]-cmd_vel[i-1])
#    diff_cmd_steering_angle.append(cmd_steering_angle[i]-cmd_steering_angle[i-1])
#print max(diff_cmd_vel), min(diff_cmd_vel)
#print max(diff_cmd_steering_angle), min(diff_cmd_steering_angle)
horizon = 10#horizon for estimate uncertainty
diff_cmd_vel_10, diff_cmd_steering_angle_10 = list(), list()
for i in xrange(horizon, len(cmd_vel_10)):
    diff_cmd_vel_10.append(cmd_vel_10[i]-cmd_vel_10[i-horizon])
    diff_cmd_steering_angle_10.append(cmd_steering_angle_10[i]-cmd_steering_angle_10[i-horizon])
#print max(diff_cmd_vel_10), min(diff_cmd_vel_10), np.mean(diff_cmd_vel_10), np.std(diff_cmd_vel_10)
#print max(diff_cmd_steering_angle_10), min(diff_cmd_steering_angle_10), np.mean(diff_cmd_steering_angle_10), np.std(diff_cmd_steering_angle_10)

#n, bins, patches = plt.hist(x=diff_cmd_vel_10, bins='auto', color='black',
#                            alpha=0.7, rwidth=0.85)
#plt.grid(axis='y', alpha=0.75)
#plt.xlabel('Velocity Difference')
#plt.ylabel('Frequency')
#plt.text(-0.5, 80, r'$\mu=7.76*10^{-5}, \sigma=0.04$')
#maxfreq = n.max()
#plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)

#n, bins, patches = plt.hist(x=diff_cmd_steering_angle_10, bins='auto', color='black',
#                            alpha=0.7, rwidth=0.85)
#plt.grid(axis='y', alpha=0.75)
#plt.xlabel('Steering Angle Difference')
#plt.ylabel('Frequency')
#plt.text(-1.2, 100, r'$\mu=0.002, \sigma=0.27$')
#maxfreq = n.max()
#plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
#plt.show()

gp_dir = 'gp_train.txt'
x_rel, y_rel, max_vel_diff, max_angle_diff = list(), list(), list(), list()
for i in xrange(len(cmd_vel_10)-horizon):
    x_rel.append(pos_x_10[i]-waypoints_x_10[i])
    y_rel.append(pos_y_10[i]-waypoints_y_10[i])
    vel_diff = []
    angle_diff = []
    for j in xrange(1, horizon):
        vel_diff.append(np.abs(cmd_vel_10[i]-cmd_vel_10[i+j]))
        angle_diff.append(np.abs(cmd_steering_angle_10[i]-cmd_steering_angle_10[i+j]))
    max_vel_diff.append(max(vel_diff))
    max_angle_diff.append(max(angle_diff))
#print max_vel_diff[0:10]
#print max_angle_diff[0:10]
x_rel = np.array(x_rel)
y_rel = np.array(y_rel)
waypoints_x_10 = np.array(waypoints_x_10)
#print len(waypoints_x_10)
waypoints_y_10 = np.array(waypoints_y_10)
waypoints_curvature_10 = np.array(waypoints_curvature_10)
max_vel_diff = np.array(max_vel_diff)
max_angle_diff = np.array(max_angle_diff)
#print len(waypoints_x_10[0:-1*horizon])
N = len(x_rel)
np.savez('gp.npz', x_rel=x_rel.reshape(N, 1), y_rel=y_rel.reshape(N, 1) , waypoints_x=waypoints_x_10[0:N].reshape(N, 1), waypoints_y=waypoints_y_10[0:N].reshape(N, 1), waypoints_curvature = waypoints_curvature_10[0:N].reshape(N, 1), max_vel_diff=max_vel_diff.reshape(N, 1), max_angle_diff=max_angle_diff.reshape(N, 1))

#with open(gp_dir, 'w') as h:
#    for i in xrange(len(cmd_vel_10)-horizon):
#        h.write(str(x_rel[i])+' '+str(y_rel[i])+' '+str(waypoints_x_10[i])+' '+str(waypoints_y_10[i])+' ' + str(waypoints_curvature_10[i]) + ' '+str(max_vel_diff[i])+' '+str(max_angle_diff[i])+'\n')
#h.close()
