#!/usr/bin/env python

import rospy
import actionlib

from pkg_task4.msg import DetectPackagesAction, DetectPackagesGoal

class DetectedPackages(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('camera_1_detect_packages',
                                                   DetectPackagesAction)
        self.packages = None

        self.client.wait_for_server()

    def done_cb(self, status, result):
        self.packages = result.packages

    def get_packages(self):
        goal = DetectPackagesGoal()
        self.client.send_goal(goal, done_cb=self.done_cb)
        self.client.wait_for_result()

        return self.packages

if __name__ == '__main__':
    rospy.init_node('node_detect_package')
    dp = DetectedPackages()
    print dp.get_packages()

    rospy.spin()
