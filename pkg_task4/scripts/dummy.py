#! /usr/bin/env python
import rospy
from UR5Controls import Ur5Controller
import yaml
from models import *


def main():
    rospy.init_node("dummy")
    robot = Ur5Controller("ur5_1")
    robot.hard_set_joint_angles([-160.9144984, 12.3438320697, -
                                 116.097055028, -76.5067822477, -18.356695945, -86.5154563036], 3)
    # robot.hard_set_joint_angles([0.706232802096, -141.981689307, -
    #                              49.9858133044, -81.2825158746, 89.6194706419, 179.984754632], 3)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Closing...")
