#!/usr/bin/env python
import rospy
from models import *
from UR5Controls import Ur5Controller
from DetectedPackages import DetectedPackages


class picker(object):
    def __init__(self):
        rospy.init_node("Node_Task4_Picker_control")
        self._ur5 = Ur5Controller("ur5_1")
        self.detected_packages = DetectedPackages()
        while not self._ur5.hard_set_joint_angles(ur5_new_starting_angles, 3) and not rospy.is_shutdown():
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

    def run(self):
        pickable_packages = self.detected_packages.get_packages()
        rospy.loginfo("\033[94mPickable_packages = \n{}\n \033[0m".format(
            str(pickable_packages)))

        picked = 0
        for package_name, _ in pickable_packages.items():
            rospy.loginfo(
                "\033[32;1mUR51: Going to pickup {}\033[0m".format(package_name))
            self.go_to_package(package_name)
            rospy.loginfo(
                "\033[32;1mUR51: Going to drop {}\033[0m".format(package_name))
            self.go_home(package_name)
            self._ur5.hard_set_joint_angles(ur5_new_starting_angles, 3)

            picked += 1
            if picked >= 9:
                break


if __name__ == "__main__":
    pick = picker()
    pick.run()
    rospy.loginfo("Picker: Picked all packages")
    rospy.spin()
