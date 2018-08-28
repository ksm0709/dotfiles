#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import PoseStamped


class DisplayVideoState(EventState):
	"""
	Display Video Stream
	"""
	
	def __init__(self, filepath=''):
		super(DisplayVideoState, self).__init__(outcomes=['done'])
		self.filepath = filepath

	def execute(self, userdata):
		return 'done'
	
	def on_enter(self, userdata):
		pass
