#!/usr/bin/env python

import rospy
from lab_ros_perception.AprilTagModule import AprilTagModule
import time
import tf2_ros
import tf2_geometry_msgs
import math

def Quaternion_toEulerianAngle(x, y, z, w):
    ysqr = y*y
    
    t0 = +2.0 * (w * x + y*z)
    t1 = +1.0 - 2.0 * (x*x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w*y - z*x)
    t2 =  1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    Z = math.degrees(math.atan2(t3, t4))
    
    
    return X, Y, Z 

def main():
    rospy.init_node('AprilTagModule_test')
    tag_module = AprilTagModule()
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    r = rospy.Rate(5)
    
    while True:
        pose = tag_module.getPoseForID(0, duration=rospy.Duration(4.0), frame_id='base')
        if pose is not None:
            p = pose.pose
            print("x:{}, y:{}, z:{}".format(p.position.x, p.position.y, p.position.z))
            yaw, pitch, roll = Quaternion_toEulerianAngle(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            print("BASE: yaw:{}, pitch:{}, roll:{}".format(yaw, pitch, roll))
            #print("BASE quad: x:{}, y:{}, z:{}, w:{}".format(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


        time.sleep(1)

if __name__ == '__main__':
    main()