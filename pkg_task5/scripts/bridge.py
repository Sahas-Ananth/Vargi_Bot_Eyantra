#!/usr/bin/env python

import datetime as dt
import json

import actionlib
import rospy

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgIotRosGoal
from pkg_ros_iot_bridge.msg import msgIotRosAction

from DetectedPackages import DetectedPackages
from models import *


class RosIoTBridge:
    def __init__(self):
        self.ac = actionlib.ActionClient("/action_ros_iot", msgIotRosAction)

        config_sheets = rospy.get_param('google_apps')
        self.sheet_script_id = config_sheets['spread_sheet_id']

        rospy.loginfo("Initialization done. Waiting for Action server...")

        self.team_id = "VB#2195"
        self.unique_id = "EndEplEa"
        self.dispatched_time = {}
        self.shipped_time = {}

        detected_packages = DetectedPackages()
        self.packages = detected_packages.get_packages()

        self.ac.wait_for_server()

    def pushtoSheets(self, **msgParams):
        goal = msgIotRosGoal()
        goal.protocol = "https"
        goal.mode = "get"
        goal.topic = self.sheet_script_id
        goal.message = json.dumps(msgParams)
        self.ac.send_goal(goal)

    def shelf_no(self, n): return str(int(n)+1)

    def dict_lookup(
            self, name, param, no=1):
        if no == 1:
            return lookup_dict[self.packages[name]][param]
        elif no == 2:
            return lookup_dict2[name][param]

    def update_sheets(self, sheet_name, order):
        # TODO: try adding lookup table 2 to order dict itself

        if sheet_name == "IncomingOrders":
            item_name = order["item"]
            self.pushtoSheets(
                id=sheet_name,
                Team_ID=self.team_id,
                Unique_ID=self.unique_id,
                Order_ID=order["order_id"],
                Order_Date_and_Time=order["order_time"],
                Item=item_name,
                Priority=self.dict_lookup(item_name, 1, 2),
                Order_Quantity=order["qty"],
                City=order["city"],
                Longitude=order["lon"],
                Latitude=order["lat"],
                Cost=self.dict_lookup(item_name, 2, 2)
            )
            rospy.loginfo("{} has been updated successfully in {}".format(
                order["order_id"], sheet_name))
        elif sheet_name == "OrdersDispatched":
            item_name = order["item"]
            self.dispatched_time[order["order_id"]] = dt.datetime.now()
            self.pushtoSheets(
                id="OrdersDispatched",
                Team_ID=self.team_id,
                Unique_ID=self.unique_id,
                Order_ID=order["order_id"],
                Dispatch_Date_and_Time=str(
                    self.dispatched_time[order["order_id"]]),
                Item=item_name,
                Priority=self.dict_lookup(item_name, 1, 2),
                Dispatch_Quantity=order["qty"],
                City=order["city"],
                Dispatch_Status="Yes",
                Cost=self.dict_lookup(item_name, 2, 2)
            )
            rospy.loginfo("{} has been updated successfully in {}".format(
                order["order_id"], sheet_name))
        elif sheet_name == "OrdersShipped":
            item_name = order["item"]
            self.shipped_time[order["order_id"]] = dt.datetime.now()
            self.pushtoSheets(
                id="OrdersShipped",
                Team_ID=self.team_id,
                Unique_ID=self.unique_id,
                Order_ID=order["order_id"],
                Shipped_Date_and_Time=str(
                    self.shipped_time[order["order_id"]]),
                Item=item_name,
                Priority=self.dict_lookup(item_name, 1, 2),
                Shipped_Quantity=order["qty"],
                City=order["city"],
                Shipped_Status="Yes",
                Cost=self.dict_lookup(item_name, 2, 2),
                Estimated_Time_of_Delivery=self.dict_lookup(item_name, 3, 2)
            )
            rospy.loginfo("{} has been updated successfully in {}".format(
                order["order_id"], sheet_name))
        elif sheet_name == "Inventory":
            names = self.packages.keys()
            names.sort()

            for name in names:
                time = dt.datetime.now()
                sku = "{}{}{}{}".format(self.packages[name][0].upper(),
                                        name[-2:],
                                        time.strftime("%m"),
                                        time.strftime("%y"))

                storage_num = "R{} C{}".format(
                    self.shelf_no(name[-2]), self.shelf_no(name[-1]))

                self.pushtoSheets(
                    id="Inventory",
                    Team_ID=self.team_id,
                    Unique_ID=self.unique_id,
                    SKU=sku,
                    Item=self.dict_lookup(name, 0),
                    Priority=self.dict_lookup(name, 1),
                    Storage_Number=storage_num,
                    Cost=self.dict_lookup(name, 2),
                    Quantity=names.count(name)
                )

                rospy.loginfo(
                    "{} sheet has been sucessfully updates".format(sheet_name))
                # TODO: Make this time independant
                rospy.sleep(2)
        elif sheet_name == "Dashboard":
            item_name = order["item"]
            diff = self.shipped_time[order["order_id"]] - \
                self.dispatched_time[order["order_id"]]
            self.pushtoSheets(
                id="Dashboard",
                Order_ID=order["order_id"],
                Item=item_name,
                Priority=self.dict_lookup(item_name, 1, 2),
                City=order["city"],
                Longitude=order["lon"],
                Latitude=order["lat"],
                Shipped="Yes",
                Dispatched="Yes",
                Order_Date_and_Time=order["order_time"],
                Dispatch_Date_and_Time=str(
                    self.dispatched_time[order["order_id"]]),
                Shipped_Date_and_Time=str(
                    self.shipped_time[order["order_id"]]),
                Time_Taken=str(diff))
            rospy.loginfo("{} has been updated successfully in {}".format(
                order["order_id"], sheet_name))
        else:
            rospy.loginfo("Error! Wrong sheet name")


if __name__ == '__main__':
    try:
        rospy.init_node("node_t5_RosIoTBridge")
        bridge = RosIoTBridge()
        # bridge.update_sheets(sheet_name="Inventory", order={})
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Closing...")
