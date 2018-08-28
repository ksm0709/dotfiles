#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Twist


class PublishCmdvelState(EventState):
	"""
	Publishes a velocity command
	"""
	
	def __init__(self, topic='/cmd_vel', linear_vel=0.0, rot_speed=0.0):
		super(PublishCmdvelState, self).__init__(outcomes=['done'])

		self._topic = topic
		self._pub = ProxyPublisher({self._topic: Twist})

		self.twist = Twist()
		self.twist.linear.x = linear_vel
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = rot_speed

	def execute(self, userdata):
		return 'done'
	
	def on_enter(self, userdata):
		self._pub.publish(self._topic, self.twist)
