#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Convert ViconBridge:TransformStamped to Vicon:PoseStamped and Vicon:Odometry
"""

import rospy
import geometry_msgs.msg
import nav_msgs.msg
import numpy
import tf



class GetPose(object):

	def __init__(self):
		rospy.init_node('transform_to_pose')
		self.loop_rate = rospy.Rate(30.0)
		rospy.Subscriber('vicon/rc_car2/rc_car2', geometry_msgs.msg.TransformStamped, self.transform_callback)
		self.pose_pub = rospy.Publisher('vicon_pose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size = 100)
		self.odom_pub = rospy.Publisher('vicon_odom', nav_msgs.msg.Odometry, queue_size = 100)
		self.twist_pub = rospy.Publisher('vicon_twist', geometry_msgs.msg.Twist, queue_size = 100)
		
		# tf_listener = tf.TransformListener()
		# tf_listener.waitForTransform('/odom', '/vicon/rc_car2/rc_car2', rospy.Time(0), rospy.Duration(10.0))
		# (trans, rot) = tf_listener.lookupTwist('/odom', '/vicon/rc_car2/rc_car2', rospy.Time(0), rospy.Duration(10.0))

		self.loop_rate.sleep()

		# self.tf_listener = tf.TransformListener()

	def transform_callback(self, trans_t):
		pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
		pose_msg.header = trans_t.header
		pose_msg.header.frame_id = 'vicon_map'
		pose_msg.pose.pose.position = trans_t.transform.translation
		pose_msg.pose.pose.orientation = trans_t.transform.rotation
		self.pose_pub.publish(pose_msg)

		odom_msg = nav_msgs.msg.Odometry()
		odom_msg.pose.pose = pose_msg.pose.pose
		odom_msg.header = trans_t.header
		odom_msg.child_frame_id = 'vicon_base_link'
		odom_msg.header.frame_id = 'vicon_map'
		self.odom_pub.publish(odom_msg)
		
		#self.tf_listener = tf.TransformListener()
		#self.odom_pub.publish(self.odom_msg)
		# print(trans_t.header.stamp.secs)
		# print(rospy.get_rostime() ,' \t', rospy.Time.now(), rospy.get_time())
		# #self.tf_listener.waitForTransform('/odom', '/vicon/rc_car2/rc_car2', rospy.get_rostime(), rospy.Duration(10.0))
		# twist_msg = geometry_msgs.msg.Twist()
		# #tf_listener = tf.TransformListener()
		# #(trans, rot) = tf_listener.lookupTwist('/odom', '/vicon/rc_car2/rc_car2', rospy.get_rostime(), rospy.Duration(0.9))
		# self.twist_pub.publish(twist_msg)

		self.loop_rate.sleep()


if __name__ == '__main__':
	get_pose = GetPose()
	rospy.spin()