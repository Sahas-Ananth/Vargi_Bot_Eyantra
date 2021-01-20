#!/usr/bin/env python
import rospy
from models import *
from UR5Controls import Ur5Controller


class picker(object):
    def __init__(self):
        rospy.init_node("Node_Task4_Picker_control")
        self._ur5 = Ur5Controller("ur5_1")
        while not self._ur5.hard_play_saved_path(self._ur5._file_path, "all0tohome", 3) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.sleep(1)

        rospy.loginfo("\033[32;1mUR5 Picker Has been initated\033[0m")

    def go_to_package(self, package_name):
        goal = "ur5_1_home_to_{}".format(package_name)
        result = self._ur5.hard_play_saved_path(self._ur5._file_path, goal, 3)
        self._ur5.gripper(package_name, True)
        return result

    def go_home(self, package_name):
        goal = "ur5_1_{}_to_home".format(package_name)
        result = self._ur5.hard_play_saved_path(self._ur5._file_path, goal, 3)
        self._ur5.gripper(package_name, False)
        self._ur5.remove_box(package_name, 0.5)
        return result


def main():
    robot = picker()
    package = "packagen{}{}"
    camera = Camera1()
    pickable_packages = camera.packages
    # robot.go_to_package("packagen21")
    # robot.go_home("packagen21")
    pickable_packages = pkg_colours

    rospy.loginfo("\033[94mPickable_packages = \n{}\n \033[0m".format(
        str(pickable_packages)))

    # NOTE: This only takes and places 00 --> 12 others will come soon
    for i in range(2):
        for j in range(3):
            package_name = package.format(str(i), str(j))
            rospy.loginfo(
                "\033[32;1mUR51: Going to pickup {}\033[0m".format(package_name))
            robot.go_to_package(package_name)
            rospy.loginfo(
                "\033[32;1mUR51: Going to drop {}\033[0m".format(package_name))
            robot.go_home(package_name)
            robot._ur5.hard_set_joint_angles(ur5_new_starting_angles, 3)


if __name__ == "__main__":
    main()
