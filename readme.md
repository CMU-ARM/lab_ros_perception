# lab_ros_perception

Python Interfaces for perception system used in the Assitive Robots Lab at CMU.

Maintainer: Zhi
Version: 0.0.1

## Current Modules

### AprilTagModule

Usage:
```
from lab_ros_perception.AprilTagModule import AprilTagModule

tag_module = AprilTagModule()
stampedPose = tag_module.getPoseForID(0)
	
```
Must launch apriltag_ros node before using this. Refer to `apriltag_kinect2.launch` for example.