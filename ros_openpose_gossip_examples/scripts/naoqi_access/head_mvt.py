#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import time
import argparse
from naoqi import ALProxy

class head_mvt():

	def __init__(self, robotIP, PORT = 9559):
		self.robotIP =  robotIP
		self.PORT = PORT


	def look_straight(self):

		motionProxy = ALProxy("ALMotion", self.robotIP, self.PORT)

		motionProxy.setStiffnesses("Head", 1.0)

		# Example showing multiple trajectories
		# Interpolates the head yaw to 1.0 radian and back to zero in 2.0 seconds
		# while interpolating HeadPitch up and down over a longer period.
		names  = ["HeadYaw","HeadPitch"]
		# Each joint can have lists of different lengths, but the number of
		# angles and the number of times must be the same for each joint.
		# Here, the second joint ("HeadPitch") has three angles, and
		# three corresponding times.
		angleLists  = [[0.0, 0.0],
					[0.0, 0.0]]
		timeLists   = [[0.2, 0.4], [ 0.2, 0.4]]
		isAbsolute  = True
		motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

		#motionProxy.setStiffnesses("Head", 0.0)



	def enableAwareness(self, enabled):
		basicAwarenessProxy = ALProxy("ALBasicAwareness", self.robotIP, self.PORT)
		basicAwarenessProxy.setEnabled(enabled)