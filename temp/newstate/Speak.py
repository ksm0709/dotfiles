#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
from std_msgs.msg import UInt8


class SpeakState(EventState):
	"""
	Genie TTS request	
	"""

	def __init__(self, text = ''):
		super(SpeakState, self).__init__(outcomes=['done'])

		self.text = text
		self.tts_topic = '/genie_tts'	
		self.state_topic = '/genie_state'
		self.tts_pub = ProxyPublisher({self.tts_topic : String})	
		self.state_sub = ProxySubscriberCached({self.state_topic : UInt8})	

	def execute(self, userdata):
		if self.state_sub.has_msg(self.state_topic) :
			
			# get genie state
			state = self.state_sub.get_last_msg(self.state_topic)
			self.state_sub.remove_last_msg(self.state_topic)

			# non-speaking state
			if state.data == 0 :
				return 'done'

	
	def on_enter(self, userdata):

		tts_text = String()
		tts_text.data = '{"ttxtext":'+self.text+',"language":0}'

		# send tts request	
		self.tts_pub.publish(self.tts_topic, tts_text)
