#!/usr/bin/env python

import rospy
import actionlib

from pkg_task4.msg import DetectPackagesAction, DetectPackagesGoal

def list_to_dict(packages):
    # Remove the end brackets
    packages = packages[1:-2]
    p_list = packages.split(',')

    # Strip whitespaces and remove the starting and ending quotes
    stripped = [p.strip()[1:-1] for p in p_list]

    packages = {}

    for package in stripped:
        name, col = package.split('=')
        packages[name] = col

    return packages

class DetectedPackages(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('camera_1_detect_packages',
                                                   DetectPackagesAction)
        self.packages = None

        self.client.wait_for_server()

    def done_cb(self, status, result):
        self.packages = result.packages

    def get_packages(self):
        while not rospy.has_param("DetectedPackages"):
            pass
        # goal = DetectPackagesGoal()
        # self.client.send_goal(goal, done_cb=self.done_cb)
        # self.client.wait_for_result()
        self.packages = rospy.get_param("DetectedPackages")

        return list_to_dict(self.packages)

if __name__ == '__main__':
    rospy.init_node('node_detect_package')
    dp = DetectedPackages()
    print dp.get_packages()

    rospy.spin()
