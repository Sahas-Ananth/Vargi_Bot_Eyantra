#!/usr/bin/env python

"""
This module queries the param server to see if there are any
detected packages.
"""

import rospy
import actionlib

class DetectedPackages(object):
    def __init__(self):
        self.packages = None

    def get_packages(self):
        """
        Returns a dictionary of packages detected by *Camera1* by
        querying the param server
        """
        while not rospy.has_param("DetectedPackages"):
            pass
        self.packages = rospy.get_param("DetectedPackages")

        return self.packages

if __name__ == '__main__':
    rospy.init_node('node_t4_detect_package')
    dp = DetectedPackages()
    print dp.get_packages()

    rospy.spin()
