#! /usr/bin/env python

import sys

import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *


# [0.706232802096, -141.981689307, -49.9858133044, -81.2825158746, 89.6194706419, 179.984754632]
# TODO: packagen02 --> home, packagen20 --> home (crash with shelf), packagen21-->home (check collisions)
# TODO: home --> packagen22 (Collision)


def package2home(package_name, robot):
    joint_values = [0.706232802096, -141.981689307, -
                    49.9858133044, -81.2825158746, 89.6194706419, 179.984754632]
    robot.gripper(package_name, True)
    file_name = "{}_{}_to_home".format(sys.argv[1], package_name)
    robot.hard_set_joint_angles(joint_values, 3)
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
    joint_angles = [-160.9144984, 12.3438320697, -
                    116.097055028, -76.5067822477, -18.356695945, -86.5154563036]
    name = "packagen22"
    home2package(joint_angles, name, robot)
    # package2home(name, robot)

    rospy.loginfo("Done.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Closing...")
