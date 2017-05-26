#!/usr/bin/env python

import rospy
from lab_ros_perception.AprilTagModule import AprilTagModule
import time

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
