# lab_ros_perception

Python Interfaces for perception system used in the Assitive Robots Lab at CMU.

Maintainer: Zhi (zhi.tan (@t) ri.cmu.edu)

Version: 0.0.2  
[Changelog](changelog.md)

To install, please do it through the .rosinstall file in [lab_scripts](https://github.com/CMU-ARM/lab_scripts)


## Current Modules

### AprilTagModule
**(Depreciated!!!!!!!!!)**
Use `ArucoTagModule` instead for AprilTags

Usage:
```
from lab_ros_perception.AprilTagModule import AprilTagModule

tag_module = AprilTagModule()
stampedPose = tag_module.getPoseForID(0)
	
```
Must launch `apriltag_ros` node before using this. Refer to `apriltag_kinect2.launch` and 'scripts/apriltag_demo.py' for example.

### ArucoTagModule
Usage:
```
from lab_ros_perception.ArucoTagModule import ArucoTagModule

tag_module = ArucoTagModule()
stampedPose = tag_module.getPoseForID(0)
stampedPose = tag_module.getPoseForIDs([0,1])
tag_module.waitForID(0, timout=)
	
```
Must launch `apriltag_ros` node before using this. Refer to `apriltag_kinect2.launch` and 'scripts/apriltag_demo.py' for example.