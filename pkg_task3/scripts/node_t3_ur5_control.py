#! /usr/bin/env python

from hrwros_gazebo.msg import LogicalCameraImage, Model
from ConveyorControl import Conveyor
from Ur5Control import Ur5
from tf import TF
from models import *
from geometry_msgs.msg import PoseStamped
from pkg_task3.msg import ur5PickupAction
from pkg_task3.msg import ur5PickupResult
from pkg_task3.msg import ur5PickupFeedback

import rospy
import actionlib
import tf2_ros

class Task3:
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
        if package == "packagen1":
            return gazebo_red_bin_angles
        elif package == "packagen2":
            return gazebo_green_bin_angles
        elif package == "packagen3":
            return gazebo_blue_bin_angles

    def _get_next_pose(self, package):
        if package == "packagen1":
            return ur5_green_box_angles
        if package == "packagen2":
            return ur5_blue_box_angles
        if package == "packagen3":
            return ur5_starting_angles

    def on_goal(self, obj_msg_goal):
        """
        Process goals and send results
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
        while tries > 0 and not self._ur5.ee_cartesian_translation(trans_x, trans_y, trans_z) and not rospy.is_shutdown():
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

        while not self._ur5.set_joint_angles(self._get_bin(package_name)) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self._ur5.gripper(package_name, False)
        self._ur5.remove_box(package_name, 0.5)

        rospy.logdebug('\033[33;1mSending result back to client\033[0m')
        obj_msg_result.success = True
        self._sas.set_succeeded(obj_msg_result)

        # Optimization for time
        # while not self._ur5.set_joint_angles(ur5_starting_angles) and not rospy.is_shutdown():
        #     rospy.sleep(0.5)
        while not self._ur5.set_joint_angles(self._get_next_pose(package_name)) and not rospy.is_shutdown():
            rospy.sleep(0.5)


if __name__ == '__main__':
    task3_solution = Task3()

    rospy.spin()