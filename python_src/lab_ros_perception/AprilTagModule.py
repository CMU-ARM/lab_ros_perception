#!/usr/bin/env python

import rospy
from apriltags_ros.msg import AprilTagDetectionArray
import time

class AprilTagModule(object):


	def _tagCallback(self, msg):

		for detected in msg.detections:
			tag_id = detected.id 

			#updated stamped pose
			self.detected_tags[tag_id] = detected.pose

	def __init__(self):
		rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self._tagCallback)
		
		#save list
		self.detected_tags = {}

		#not sure if we need to spin here

	def getPoseForID(self, id_):
		
		if id_ in self.detected_tags:
			return self.detected_tags[id_]
		else:
			rospy.loginfo("ID never been seen")
			return None



def main():
	rospy.init_node('AprilTagModule_test')
	tag_module = AprilTagModule()

	r = rospy.Rate(5)

	while True:
		pose = tag_module.getPoseForID(0)
		if pose is not None:
			p = pose.pose
			print("x:{}, y:{}, z:{}".format(p.position.x, p.position.y, p.position.z))
		time.sleep(1)

if __name__ == '__main__':
	main()
