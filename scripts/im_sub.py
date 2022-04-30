#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: nosaka
# @brief: marker check test

# General
import numpy as np

# ROS
import rospy
import tf
from visualization_msgs.msg import InteractiveMarkerFeedback

class InteractiveMarkerSub:
    def __init__(self):
        # common
        self.PI = np.pi
        # iMarker Callback
        self.pos = []
        self.quat = []
        self.euler = []
        # sub
        rospy.Subscriber("/simple_marker/feedback",
            InteractiveMarkerFeedback,
            self.iMarker_CB)

    def iMarker_CB(self, msg):
        self.pos = msg.pose.position
        self.quat = msg.pose.orientation
        self.euler = tf.transformations.euler_from_quaternion((self.quat.x,
            self.quat.y,
            self.quat.z,
            self.quat.w))

    def rad2deg(self, euler):
        return euler * (180.0 / self.PI)

    def normalize(self, euler):
        return euler / self.PI

    def main(self):
        while not rospy.is_shutdown():
            try:
                self.euler = np.array(self.euler) # (3,) (vec,)
                #self.euler = self.euler.reshape(1, 3) # (1, 3) (batch, vec)
                #self.euler = self.normalize(self.euler)
            except:
                pass

if __name__ == "__main__":
    rospy.init_node("interactive_marker_sub", anonymous = True)
    ims = InteractiveMarkerSub()
    while not rospy.is_shutdown():
        print(ims.euler)
        rospy.sleep(0.5)
