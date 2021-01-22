#!/usr/bin/env python

import rospy
import actionlib

from pkg_task4.msg import DetectPackagesAction, DetectPackagesGoal

class DetectedPackages(object):
    def __init__(self):
        self.packages = None

    def get_packages(self):
        while not rospy.has_param("DetectedPackages"):
            pass
        # goal = DetectPackagesGoal()
        # self.client.send_goal(goal, done_cb=self.done_cb)
        # self.client.wait_for_result()
        self.packages = rospy.get_param("DetectedPackages")

        return self.packages

if __name__ == '__main__':
    rospy.init_node('node_t4_detect_package')
    dp = DetectedPackages()
    print dp.get_packages()

    rospy.spin()
