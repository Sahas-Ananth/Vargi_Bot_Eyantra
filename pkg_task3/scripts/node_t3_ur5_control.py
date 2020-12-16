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
This modules represent the UR5 arm. It receives goals from the
logical camera node on the *ur5_pickup* topic and manipulates
the arm toward the packages and bins.
"""

import rospy
import actionlib
import tf2_ros

from pkg_task3.msg import ur5PickupAction, ur5PickupResult
from ConveyorControl import Conveyor
from Ur5Control import Ur5
from geometry_msgs.msg import PoseStamped
from tf import TF
from models import *

class Task3(object):
    """
        Task3 class contains an Ur5 arm controller instance and
        on receiving goals from the camera node, manipulates the
        ur5 instance to location, pickups the package and drop
        in the correct bin.
    """
    def __init__(self):
        rospy.init_node("node_t3_ur5_control")

        # Reference frames used in TF
        self._ref_frame = "logical_camera_2_{}_frame"
        self._target_frame = "ur5_ee_link"

        # Offset to be maintained from the package to properly
        # pickup the object
        self._delta = 0.11 # vacuum_gripper_width + (box_length/2)

        # UR5 control instance
        self._ur5 = Ur5()
        # TF instance
        self._tf = TF()
        # Instance to control the conveyor belt
        self._conveyor = Conveyor()

        self._sas = actionlib.SimpleActionServer("/ur5_pickup",
                                                 ur5PickupAction,
                                                 execute_cb=self.on_goal,
                                                 auto_start=False)

        # Move the arm to a proper starting position
        while not self._ur5.set_joint_angles(ur5_new_starting_angles) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.sleep(1)

        rospy.loginfo("\033[32;1mUR5 Control: Starting Simple Action Server.\033[0m")
        self._sas.start()

    def _get_bin(self, package):
        """
        _get_bin: returns the correct bin for package passed.
                  package: Can be one of the 3 packages format
                  *packagen{1, 2, 3}*.
        """
        if package == "packagen1":
            return gazebo_red_bin_angles
        elif package == "packagen2":
            return gazebo_green_bin_angles
        elif package == "packagen3":
            return gazebo_blue_bin_angles
        return None

    def _get_next_pose(self, package):
        """
        _get_next_pose: This is used as an optimization step.
                        Returns the next pose that is closest
                        to the next package to be picked up.
        """
        if package == "packagen1":
            return ur5_green_box_angles
        if package == "packagen2":
            return ur5_blue_box_angles
        return ur5_starting_angles

    def on_goal(self, obj_msg_goal):
        """
        on_goal: Callback on receiving a goal from the camera node.
                 It calculates the offset between the end effector
                 and package and manipulates the arm with UR5 controller
                 instance.
        """
        rospy.loginfo("\033[32;1mUR5 Control: Received a Goal from Client.\033[0m")
        rospy.loginfo("\033[32;1mpackage: {}\033[0m".format(obj_msg_goal.package_name))

        package_name = obj_msg_goal.package_name

        trans = self._tf.lookup_transform(self._ref_frame.format(package_name), self._target_frame)

        trans_x = -trans.transform.translation.x
        trans_y = -trans.transform.translation.y
        trans_z = -trans.transform.translation.z + self._delta

        # Send Result to the Client
        obj_msg_result = ur5PickupResult()

        tries = 3
        while tries > 0 and not self._ur5.ee_cartesian_translation(trans_x, trans_y, trans_z) \
              and not rospy.is_shutdown():
            rospy.loginfo('\033[33;1mMoving to package failed, Trying again\033[0m')
            rospy.sleep(0.5)
            tries -= 1
        if tries == 0:
            rospy.loginfo('\033[33;1mAll tries failed\033[0m')
            obj_msg_result.success = False
            self._sas.set_succeeded(obj_msg_result)
            return

        trans = self._tf.lookup_transform("world", self._ref_frame.format(package_name))

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = trans.transform.translation
        pose.pose.orientation = trans.transform.rotation

        self._ur5.add_box(package_name, pose, box_length, 0.5)
        self._ur5.gripper(package_name, True)

        while not self._ur5.set_joint_angles(self._get_bin(package_name)) \
              and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self._ur5.gripper(package_name, False)
        self._ur5.remove_box(package_name, 0.5)

        rospy.logdebug('\033[33;1mSending result back to client\033[0m')
        obj_msg_result.success = True
        self._sas.set_succeeded(obj_msg_result)

        # Optimization for time
        # while not self._ur5.set_joint_angles(ur5_starting_angles) and not rospy.is_shutdown():
        #     rospy.sleep(0.5)
        while not self._ur5.set_joint_angles(self._get_next_pose(package_name)) \
              and not rospy.is_shutdown():
            rospy.sleep(0.5)


if __name__ == '__main__':
    Task3()

    rospy.spin()
