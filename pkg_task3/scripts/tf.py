#! /usr/bin/env python

# Copyright <2020> <Niteesh G S>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
TF Module
"""
import rospy
import tf2_ros
import tf2_msgs.msg

class TF(object):
    """
    This class can be used to obtain the transform between
    different frames.
    """
    def __init__(self):
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def lookup_transform(self, arg_frame_1, arg_frame_2):
        """
        lookup_transform: Return the transform between the arg_frame1
                          arg_frame2.
                          *arg_frame1*: Target frame.
                          *arg_frame2*: Reference frame.
        """
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
