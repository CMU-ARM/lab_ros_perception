# lab_ros_perception

Python Interfaces for perception system used in the Assitive Robots Lab at CMU.

Maintainer: Zhi (zhi.tan (@t) ri.cmu.edu)

Version: 0.0.2  
[Changelog](changelog.md)

## Current Modules

### AprilTagModule

Usage:
```
from lab_ros_perception.AprilTagModule import AprilTagModule

tag_module = AprilTagModule()
stampedPose = tag_module.getPoseForID(0)
	
```
Must launch `apriltag_ros` node before using this. Refer to `apriltag_kinect2.launch` and 'scripts/apriltag_demo.py' for example.
