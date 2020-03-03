#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Carnegie Mellon University and HRL Laboratories, LLC 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Edward Ahn, Aleksey Nogin

An interface for training a local planner with RC Car
"""

import rospy
import math
import nav_msgs.msg
import ackermann_msgs.msg

from aa_simulation.envs.ros.base import ROS_Base

class ROS_RCCar(ROS_Base):
    """
    An interface for training a local planner with RC Car
    """

    # Odometry message period, s
    DT = 0.035 # Measuted value for the RC Car

    def __init__(self, params):
        super(ROS_RCCar, self).__init__(params)

        self.publisher = rospy.Publisher('commands/keyboard',
                ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('ekf_localization/odom', nav_msgs.msg.Odometry,
                self._odometry_callback)

    def publish_action(self, action):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.drive.speed = action[0]
        msg.drive.steering_angle = action[1]
        msg.header.stamp = self._sensor_stamp
        self.publisher.publish(msg)
