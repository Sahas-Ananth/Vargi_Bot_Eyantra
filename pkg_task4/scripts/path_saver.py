#! /usr/bin/env python

import sys

import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *


# [0.706232802096, -141.981689307, -49.9858133044, -81.2825158746, 89.6194706419, 179.984754632]
# TODO: home <--> packagen{22,30,31,32}


def package2home(package_name, robot):
    # joint_values = ur5_new_starting_angles
    joint_values = ur5_starting_angles
    robot.gripper(package_name, True)
    file_name = "{}_{}_to_home".format(sys.argv[1], package_name)
    robot.hard_set_joint_angles(joint_values, 3)
    # robot.go_to_pose(ur5_starting_pose)
    robot.save_prev_path(robot._file_path, file_name)
    robot.gripper(package_name, False)
    robot.remove_box(package_name)


def home2package(joint_angle_list, package_name, robot):
    joint_values = joint_angle_list
    file_name = "{}_home_to_{}".format(sys.argv[1], package_name)
    robot.hard_set_joint_angles(joint_values, 3)
    robot.save_prev_path(robot._file_path, file_name)


def main():
    rospy.init_node("Path_Saver")
    robot = Ur5Controller(sys.argv[1])
    joint_angles = packagen21_angles
    name = "packagen21"
    # home2package(joint_angles, name, robot)
    package2home(package_name=name, robot=robot)

    rospy.loginfo("Done.")


if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing...")
