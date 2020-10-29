#!/usr/bin/env python2

import json

import rospy
import actionlib

from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_ros_iot_bridge.msg import msgIotRosAction
from pkg_ros_iot_bridge.msg import msgIotRosGoal
from pkg_ros_iot_bridge.msg import msgMqttSub


class TurtleClient:
    """
    This action client that sends goals to the simple action server
    to make the turtle draw a hexagon
    """

    def __init__(self):
        self.simple_client = actionlib.SimpleActionClient("/action_turtle",
                                                          msgTurtleAction)
        self.action_client = actionlib.ActionClient("/action_ros_iot",
                                                    msgIotRosAction)

        # Load params
        config_iot = rospy.get_param('config_iot')
        config_sheets = rospy.get_param('google_apps')
        self._mqtt_ros_topic = config_iot['mqtt']['topic_pub']
        self._mqtt_iot_topic = config_iot['mqtt']['topic_sub']
        self._mqtt_ros_sub_topic = config_iot['mqtt']['sub_cb_ros_topic']
        self._sheet_script_id = config_sheets['spread_sheet_id']

        # Task1: Data also to be published on eyantra sheet
        self._sheet_eyantra_id = 'AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk'

        # Subscribe to the topic where MQTT msgs are published.
        self.iot_sub = rospy.Subscriber(self._mqtt_ros_sub_topic,
                                        msgMqttSub, self.process_iot_msg,
                                        queue_size=100)

        rospy.loginfo("TurtleClient: Waiting for Simple Action Server")
        self.simple_client.wait_for_server()
        rospy.loginfo("TurtleClient: Simple Action Server up")

        rospy.loginfo("TurtleClient: Waiting for Action Server")
        self.action_client.wait_for_server()
        rospy.loginfo("TurtleClient: Action server up")

    def send_goal(self, arg_dis, arg_angle):
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        self.simple_client.send_goal(goal, done_cb=self.done_callback,
                                     feedback_cb=self.feedback_callback)

        rospy.loginfo('Goal: "dist = {}, ang = {}" sent'.format(arg_dis,
                                                                arg_angle))

    def done_callback(self, status, result):
        # Callback for result
        mqtt_goal = msgIotRosGoal()
        mqtt_goal.protocol = "mqtt"
        mqtt_goal.mode = "pub"
        mqtt_goal.topic = self._mqtt_ros_topic

        msg = "Received result: {}, {}, {}".format(result.final_x,
                                                   result.final_y,
                                                   result.final_theta)

        mqtt_goal.message = msg
        self.action_client.send_goal(mqtt_goal)

        # Publish to team sheet
        https_goal1 = msgIotRosGoal()
        https_goal1.protocol = "https"
        https_goal1.mode = "get"
        https_goal1.topic = self._sheet_script_id
        params = {'id': 'Sheet1',
                  'turtle_x': result.final_x,
                  'turtle_y': result.final_y,
                  'turtle_theta': result.final_theta}
        https_goal1.message = json.dumps(params)
        self.action_client.send_goal(https_goal1)

        # Publish to eyantra sheet
        https_goal2 = msgIotRosGoal()
        https_goal2.protocol = "https"
        https_goal2.mode = "get"
        https_goal2.topic = self._sheet_eyantra_id
        params = {'id': 'task1',
                  'team_id': 'VB_2195',
                  'unique_id': 'EndEplEa',
                  'turtle_x': result.final_x,
                  'turtle_y': result.final_y,
                  'turtle_theta': result.final_theta}
        https_goal2.message = json.dumps(params)
        self.action_client.send_goal(https_goal2)

        rospy.loginfo(msg)

    def feedback_callback(self, feedback):
        # Callback for feedback
        rospy.logdebug("feedback: {}, {}, {}".format(feedback.cur_x,
                                                     feedback.cur_y,
                                                     feedback.cur_theta))

    def process_iot_msg(self, msg):
        rospy.loginfo("Got message: \"%s\" from %s" % (msg.topic, msg.message))

        if msg.topic == self._mqtt_iot_topic and msg.message == "start":
            rospy.loginfo("Starting to draw hexagon")
            self.draw_hexagon()

    def draw_hexagon(self):
        # Sends the required goals to draw a hexagon
        sides = 0

        self.send_goal(2, 0)
        self.simple_client.wait_for_result()
        while (sides < 5):
            self.send_goal(2, 60)
            self.simple_client.wait_for_result()
            sides += 1


def main():
    rospy.init_node("node_iot_action_client_turtle")

    turtleclient = TurtleClient()

    rospy.spin()


if __name__ == "__main__":
    main()
