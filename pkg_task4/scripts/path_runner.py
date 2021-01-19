#! /usr/bin/env python

import os
import sys

import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *


def main():
    rospy.init_node("Path_player")
    robot = Ur5Controller(sys.argv[1])
    file_name = "ur5_2_all0tohome"
    robot.hard_play_saved_path(robot._file_path, file_name, 3)
    #file_name = "ur5_2_home_to_redbin"
    #robot.hard_play_saved_path(robot._file_path, file_name, 3)
    rospy.loginfo("Done.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("closing...")
