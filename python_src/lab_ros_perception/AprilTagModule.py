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
		
		if self.checkID(id_):
			return self.detected_tags[id_]
		else:
			rospy.loginfo("ID never been seen")
			return None
		
	def getPosesForIDs(self, ids):
		"""
		Returns the list of last stamped pose for the aprilTag for the given ids. Individual element will be None if 
		not seen before
		
		Parameters
		----------
		ids : int[]
			list of IDs to be search for

		Returns:
		geomtery/StampedPose[]
			list of StampedPose for the given ID last seen by April Tag Node
		"""
		
		pose_list = []
		for id_ in ids:
			pose_list.append(self.getPoseForID(id_))
		return pose_list
	
	def checkID(self, id_):
		"""
		Check whether the module has seen the ID
		
		Parameters
		----------
		id_ : int
			ID of the tag looking for

		Returns:
		Bool
			True if ID has been seen, False otherwise
		"""
		return id_ in self.detected_tags
	
	def getTimeForID(self, ids):
		"""
		Returns the time stamp of the AprilTag was last seen, none otherwise.
		
		Parameters
		----------
		id_ : int
			ID of the tag
			
		Returns:
		Time
			Two-integer timestamp of the last time the AprilTag was seen

		"""
		if self.checkID(id_):
			return self.detected_tags[id_].header.stamp
		else:
			rospy.loginfo("ID never been seen")
			return None
		
