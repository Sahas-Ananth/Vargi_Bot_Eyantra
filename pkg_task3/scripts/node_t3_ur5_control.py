#! /usr/bin/env python

from hrwros_gazebo.msg import LogicalCameraImage, Model
from ConveyorControl import Conveyor
from Ur5Control import Ur5
from tf import TF
from models import *

import rospy
import tf2_ros

gazebo_packages = []

def camera_callback(msg):
    pass

class Task3:
    def __init__(self):
        rospy.init_node("node_t3_ur5_control")

        self._ref_frame = "world"
        self._target_frame = "logical_camera_2_packagen1_frame"

        self._delta = vacuum_gripper_width + (box_length/2)

        self._ur5 = Ur5()
        self._tf = TF()

        self._ur5.go_to_predefined_pose("straightUp")
        rospy.sleep(2)


    def run(self):
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, camera_callback)

        trans = self._tf.lookup_transform(self._ref_frame, self._target_frame)

        ur5_package = geometry_msgs.msg.Pose()
        ur5_package.position.x = trans.transform.translation.x                                                                                                                       
        ur5_package.position.y = trans.transform.translation.y
        ur5_package.position.z = trans.transform.translation.z + self._delta
        ur5_package.orientation.x = -0.5
        ur5_package.orientation.y = -0.5
        ur5_package.orientation.z = 0.5
        ur5_package.orientation.w = 0.5

        print '--------------------'
        print ur5_package
        print '--------------------'

        print 'goto red bin?'
        raw_input()
        while not self._ur5.go_to_pose(gazebo_red_bin) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        print 'goto green bin?'
        raw_input()
        while not self._ur5.go_to_pose(gazebo_green_bin) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        print 'goto blue bin?'
        raw_input()
        while not self._ur5.go_to_pose(gazebo_blue_bin) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        return

        self._ur5.gripper(True)

        self._ur5.go_to_pose(gazebo_red_bin)

        self._ur5.gripper(False)

        self._ur5.go_to_predefined_pose("allZeros")

        rospy.spin()

if __name__ == '__main__':
    task3_solution = Task3()
    task3_solution.run()