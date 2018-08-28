#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String


class PublishStringState(EventState):
    '''
	Publishes a string (std_msgs/String) message on a given topic name.
	'''

    def __init__(self, topic, text=''):
        super(PublishStringState, self).__init__(outcomes=['done'])

        self._topic = topic
        self._pub = ProxyPublisher({self._topic: String})

        self.text = text

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = String()
        val.data = self.text
        self._pub.publish(self._topic, val)
