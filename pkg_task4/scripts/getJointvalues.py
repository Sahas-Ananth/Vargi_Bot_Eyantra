#! /usr/bin/env python

import os
import sys

import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *

def main():
    rospy.init_node("GetJointAngles")
    robot = Ur5Controller(sys.argv[1])
    robot.print_joint_angles()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Closing...")