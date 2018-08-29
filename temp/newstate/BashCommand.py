#!/usr/bin/env python

from flexbe_core import EventState, Logger
import subprocess


class BashCommandState(EventState):
	"""
	Run bash shell command 	
	"""
	
	def __init__(self, cmd=''):
		super(BashCommandState, self).__init__(outcomes=['done'])
		self.cmd = cmd

	def execute(self, userdata):
		subprocess.call(self.cmd, shell=True)
		return 'done'

# vlc ~/Downloads/hello.mp4
# vlc 