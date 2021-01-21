#! /usr/bin/env python

import rospy
import actionlib
import tf2_ros

from pkg_task4.msg import task4Action, task4Result
from ConveyorControl import Conveyor
from UR5Controls import Ur5Controller
from geometry_msgs.msg import PoseStamped
from tf import TF
from models import *


class Sorter(object):
    def __init__(self):
        rospy.init_node("Node_Task4_Sorter_control")
        # Reference frames used in TF
        self._ref_frame = "logical_camera_2_{}_frame"
        self._target_frame = "ur5_2_tf/ur5_ee_link"

        # Offset to be maintained from the package to properly
        # pickup the object
        self._delta = 0.11  # vacuum_gripper_width + (box_length/2)

        # UR5 control instance
        self._ur5 = Ur5Controller("ur5_2")
        # TF instance
        self._tf = TF()
        # Instance to control the conveyor belt
        self._conveyor = Conveyor()

        self._sas = actionlib.SimpleActionServer(
            "/ur5_pkg_sorter", task4Action, execute_cb=self.on_goal, auto_start=False)
        while not self._ur5.hard_set_joint_angles(ur5_new_starting_angles, 3) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.sleep(1)

        rospy.loginfo(
            "\033[32;1mUR5 Sorter: Starting Simple Action Server.\033[0m")
        self._sas.start()

    def _get_bin(self, colour):
        """
        _get_bin: returns the correct bin for package passed.
                  package: Can be one of the 3 packages format
                  *packagen{1, 2, 3}*.
        """
        if colour == "red":
            return self._ur5.hard_set_joint_angles(gazebo_red_bin_angles, 3)
        elif colour == "yellow":
            return self._ur5.hard_set_joint_angles(gazebo_green_bin_angles, 3)
        elif colour == "green":
            return self._ur5.hard_set_joint_angles(gazebo_blue_bin_angles, 3)

    def on_goal(self, obj_msg_goal):
        """
        on_goal: Callback on receiving a goal from the camera node.
                 It calculates the offset between the end effector
                 and package and manipulates the arm with UR5 controller
                 instance.
        """
        rospy.loginfo(
            "\033[32;1mUR5 Sorter: Received a Goal from Client.\033[0m")
        rospy.loginfo("\033[32;1mpackage: {} with colour: {}\033[0m".format(
            obj_msg_goal.package_name, obj_msg_goal.colour))

        package_name = obj_msg_goal.package_name
        package_colour = obj_msg_goal.colour
        package_pose = obj_msg_goal.package_pose

        robot_pose = obj_msg_goal.ur5_pose

        # trans = self._tf.lookup_transform(
        #     self._ref_frame.format(package_name), self._target_frame)

        # trans_x = 0
        # trans_y = (package_pose.position.x -
        #            robot_pose.position.y) - package_pose.position.y
        # trans_z = 0

        # # trans_x = -trans.transform.translation.x
        # # trans_y = -trans.transform.translation.y
        # # trans_z = -trans.transform.translation.z + self._delta

        # # Send Result to the Client
        obj_msg_result = task4Result()

        # tries = 3
        # while tries > 0 and not self._ur5.ee_cartesian_translation(trans_x, trans_y, trans_z) and not rospy.is_shutdown():
        #     rospy.loginfo(
        #         '\033[33;1mMoving to package failed, Trying again\033[0m')
        #     rospy.sleep(0.5)
        #     tries -= 1
        # if tries == 0:
        #     rospy.loginfo('\033[33;1mAll tries failed\033[0m')
        #     obj_msg_result.success = False
        #     self._sas.set_succeeded(obj_msg_result)
        #     return

        trans = self._tf.lookup_transform(
            "world", self._ref_frame.format(package_name))

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = trans.transform.translation
        pose.pose.orientation = trans.transform.rotation

        self._ur5.add_box(package_name, pose, box_length, 0.5)
        self._ur5.gripper(package_name, True)
        rospy.loginfo("Gripper Activated")

        while not self._get_bin(package_colour) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self._ur5.gripper(package_name, False)
        self._ur5.remove_box(package_name, 0.5)
        rospy.loginfo("Gripper Dectivated")
        self._ur5.set_joint_angles(ur5_new_starting_angles)

        rospy.logdebug('\033[33;1mSending result back to client\033[0m')
        obj_msg_result.success = True
        self._sas.set_succeeded(obj_msg_result)


if __name__ == '__main__':
    try:
        rospy.sleep(10)
        Sorter()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
