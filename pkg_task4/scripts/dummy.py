#! /usr/bin/env python
import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *


def main():
    rospy.init_node("dummy")
    robot = Ur5Controller("ur5_1")
    # robot.hard_set_joint_angles(packagen02_angles, 3)
    # robot.gripper("packagen02", True)
    # robot.hard_play_saved_path(
    # robot._file_path, "ur5_1_packagen02_to_intermediate_pose", 3)
    robot.hard_set_joint_angles(ur5_new_starting_angles, 3)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Closing...")
