#!/usr/bin/env python

import rospy
from apriltags_ros.msg import AprilTagDetectionArray
import time

class AprilTagModule(object):
	"""Wrapper for AprilTag Node
	
	Wrapper for AprilTag Node, provide easy methods to check for april tag items

	"""

	def __init__(self):
		rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self._tagCallback)
		
		#dict that stored all seen tags
		self.detected_tags = {}
		

	def _tagCallback(self, msg):

		for detected in msg.detections:
			tag_id = detected.id 

			#updated stamped pose
			self.detected_tags[tag_id] = detected.pose


	def getPoseForID(self, id_):
		"""
		Returns the last stamped pose for the aprilTag with given id if detected. Return none
		if it never seen tags with that id.
		
		Parameters
		----------
		id_ : int
			ID of the tag looking for

		Returns:
		geomtery/StampedPose
			StampedPose when the ID was last seen by the April Tag Node

		"""
		
		if id_ in self.detected_tags:
			return self.detected_tags[id_]
		else:
			rospy.loginfo("ID never been seen")
			return None