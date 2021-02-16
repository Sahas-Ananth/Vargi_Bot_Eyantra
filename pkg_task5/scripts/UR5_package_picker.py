#!/usr/bin/env python

import json

import rospy
from std_msgs.msg import String
from pkg_ros_iot_bridge.msg import msgMqttSub

from models import *
from UR5Controls import Ur5Controller
from DetectedPackages import DetectedPackages
from bridge import RosIoTBridge

"""
This module represents the UR5 Arm 1.
"""


class Picker(object):
    """
    Picker: Represents the UR5 Arm 1. This gets the colour of the detected
    packages from the param server, pick and places these packages onto
    the conveyor.
    """

    def __init__(self):
        rospy.init_node("node_t4_picker_control")
        self.sub = rospy.Subscriber(
            "/ros_iot_bridge/mqtt/incoming_orders", msgMqttSub, self.get_orders)

        self._ur5 = Ur5Controller("ur5_1")

        self.detected_packages = DetectedPackages()

        config_iot = rospy.get_param('config_iot')
        self._mqtt_ros_topic = config_iot['mqtt']['topic_pub']
        self._mqtt_iot_topic = config_iot['mqtt']['topic_sub']
        self._mqtt_ros_sub_topic = config_iot['mqtt']['sub_cb_ros_topic']
        self.dict3 = rospy.get_param('Dict3')

        self.curr_order_pub = rospy.Publisher(
            name="Current_order", data_class=String, queue_size=5)

        self.picked_packages = []
        self.orders = []
        self.ssupdater = RosIoTBridge()

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

    def get_orders(self, data):
        order = json.loads(data.message)
        self.orders.append(order)
        self.ssupdater.update_sheets("IncomingOrders", order)
        self.orders = sorted(
            self.orders, key=lambda i: i['item'], reverse=True)
        self.run()

    def run(self):
        """
        Starts picking up the packages.
        """

        # Get List of all detected packages as dict with their name as key
        # and colour as value
        order = self.orders.pop(0)
        pickable_packages = self.dict3[lookup_dict2[order["item"]][0]]
        picked = 0

        for i in range(len(pickable_packages)):
            if pickable_packages[i] not in self.picked_packages:
                rospy.loginfo(
                    "\033[32;1mUR51: Going to pickup {}\033[0m".format(pickable_packages[i]))
                self.go_to_package(pickable_packages[i])
                rospy.loginfo(
                    "\033[32;1mUR51: Going to drop {}\033[0m".format(pickable_packages[i]))
                self.go_home(pickable_packages[i])
                self._ur5.hard_set_joint_angles(ur5_new_starting_angles, 3)

                self.picked_packages.append(pickable_packages[i])
                self.ssupdater.update_sheets("OrdersDispatched", order)
                self.curr_order_pub.publish(json.dumps(order))
                picked = 1
                if picked:
                    break
            else:
                continue


if __name__ == "__main__":
    rospy.loginfo("UR5Picker: Waiting 10s for gazebo to load all packages")
    rospy.sleep(10)
    picker = Picker()
    rospy.spin()
    # picker.run()
    rospy.loginfo("Picker: Picked all packages")
    rospy.spin()
