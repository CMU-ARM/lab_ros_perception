#!/usr/bin/env python

import rospy
from apriltags_ros.msg import AprilTagDetectionArray
import time
import tf2_ros
import tf2_geometry_msgs
import numpy as np


class AprilTagModule(object):
    """Wrapper for AprilTag Node
    Wrapper for AprilTag Node, provide easy methods to check for april tag items

    """
    def __init__(self):
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._tagCallback)
        #dict that stored all seen tags
        self.detected_tags = {}
        self._tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self._tf_buffer)

    def _tagCallback(self, msg):

        for detected in msg.detections:
            tag_id = detected.id 

            #updated stamped pose
            self.detected_tags[tag_id] = detected.pose


    def getPoseForID(self, id_, duration=None, frame_id=None):
        """
        Returns the last stamped pose for the aprilTag with given id if detected. Return none
        if it never seen tags with that id.
        
        Parameters
        ----------
        id_ : int
            ID of the tag looking for

        duration : ros::Duration
            The maximum time between current and the actual pose. None if don't care

        frame_id : string
            The name of the frame to be transformed to

        Returns:
        geomtery/StampedPose
            StampedPose when the ID was last seen by the April Tag Node

        """
        
        if self.checkID(id_):
            saved_pose = self.detected_tags[id_]
            if duration is not None:
                if saved_pose.header.stamp + duration < rospy.Time.now():
                    # invalid time as it is too old
                    rospy.loginfo("Saved Pose too old")
                    return None

            #transform frame
            if frame_id is not None:
                #TODO add exception, maximum timeout etc
                stamped_transform = self._tf_buffer.lookup_transform('base', saved_pose.header.frame_id, rospy.Time(), rospy.Duration(1.0))
                saved_pose = tf2_geometry_msgs.do_transform_pose(saved_pose, stamped_transform)
                
            return saved_pose
        else:
            rospy.loginfo("ID never been seen")
            return None
        
    def getPosesForIDs(self, ids, duration=None, frame_id=None):
        """
        Returns the list of last stamped pose for the aprilTag for the given ids. Individual element will be None if 
        not seen before
        
        Parameters
        ----------
        ids : int[]
            list of IDs to be search for

        duration : ros::Duration
            The maximum time between current and the actual pose. None if don't care

        frame_id : string
            The name of the frame to be transformed to

        Returns:
        geomtery/StampedPose[]
            list of StampedPose for the given ID last seen by April Tag Node
        """
        
        pose_list = []
        for id_ in ids:
            pose_list.append(self.getPoseForID(id_, duration, frame_id))
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
    
    def waitForID(self, id, timeout=0):

        r = rospy.Rate(10)
        while id not in self.detected_tags:
            r.sleep()
        