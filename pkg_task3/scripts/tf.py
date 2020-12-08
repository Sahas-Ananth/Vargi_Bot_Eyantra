#! /usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg

class TF:

    def __init__(self):
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def lookup_transform(self, arg_frame_1, arg_frame_2):
        while True:
            try:
                trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
                # rospy.loginfo( "\n" +
                #               "Translation: \n" +
                #               "x: {} \n".format(trans.transform.translation.x) +
                #               "y: {} \n".format(trans.transform.translation.y) +
                #               "z: {} \n".format(trans.transform.translation.z) +
                #               "\n" +
                #               "Orientation: \n" +
                #               "x: {} \n".format(trans.transform.rotation.x) +
                #               "y: {} \n".format(trans.transform.rotation.y) +
                #               "z: {} \n".format(trans.transform.rotation.z) +
                #               "w: {} \n".format(trans.transform.rotation.w) )
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")
