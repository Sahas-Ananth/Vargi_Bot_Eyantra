#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import json

import rospy
import actionlib
import threading

from pkg_ros_iot_bridge.msg import msgIotRosAction
from pkg_ros_iot_bridge.msg import msgIotRosGoal
from pkg_ros_iot_bridge.msg import msgIotRosResult
from pkg_ros_iot_bridge.msg import msgIotRosFeedback

from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot


class IotRosBridgeActionServer:
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgIotRosAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
        * self.on_goal - It is the fuction pointer which points to a
        function which will be called when the Action Server receives a Goal.

        * self.on_cancel - It is the fuction pointer which points to a function
        which will be called when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        param_config_mqtt = param_config_iot['mqtt']
        self._mqtt_server_url = param_config_mqtt['server_url']
        self._mqtt_server_port = param_config_mqtt['server_port']
        self._mqtt_sub_topic = param_config_mqtt['topic_sub']
        self._mqtt_qos = param_config_mqtt['qos']
        self._mqtt_sub_cb_ros_topic = param_config_mqtt['sub_cb_ros_topic']
        rospy.loginfo(param_config_mqtt)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on
        # a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub)
        # to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._mqtt_sub_cb_ros_topic,
                                               msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is
        # defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there
        # is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._mqtt_server_url,
                                              self._mqtt_server_port,
                                              self._mqtt_sub_topic,
                                              self._mqtt_qos)
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def mqtt_sub_callback(self, client, userdata, message):
        """
        This is a callback function for MQTT Subscriptions
        """
        payload = str(message.payload.decode("utf-8"))

        rospy.loginfo("[MQTT SUB CB] Message: " + payload)
        rospy.loginfo("[MQTT SUB CB] Topic: " + message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == "mqtt"):

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client
                # (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points
                # to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()
            else:
                goal_handle.set_rejected()
                return
        elif(goal.protocol == "https"):
            if (goal.mode == "get"):
                goal_handle.set_accepted()
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()
            else:
                goal_handle.set_rejected()
        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        """
        This function is called is a separate thread to process Goal.
        """
        result = msgIotRosResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if(goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if(goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._mqtt_server_url,
                                       self._mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._mqtt_qos)

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._mqtt_server_url,
                                                      self._mqtt_server_port,
                                                      goal.topic,
                                                      self._mqtt_qos)
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False
        elif(goal.protocol == "https"):
            rospy.logwarn("HTTPS")

            if (goal.mode == "get"):
                rospy.logwarn("HTTPS GET Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic + " > " + goal.message)
                params = json.loads(goal.message)
                ret = iot.update_sheets(goal.topic, params)

                if (ret is True):
                    rospy.loginfo("HTTPS GET Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("HTTPS GET Failed.")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def on_cancel(self, goal_handle):
        """
        This function will be called when Goal Cancel request
        is send to the Action Server
        """
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Received cancel request for goal %d" % goal_id)


# Main
def main():
    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = IotRosBridgeActionServer()

    rospy.spin()


if __name__ == '__main__':
    main()
