#!/usr/bin/env python
"""
Simple action server
This server controls the turtle as the goal recevied.

Action name: /action_turtle

Controls:
    move_straigth(x): x is distance to be moved
    rotate(x): x is angle(degress) to be rotated
"""
import math

import rospy
import actionlib

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleResult
from pkg_task1.msg import msgTurtleFeedback


class SimpleActionServerTurtle:
    """
    An instance simple action server that controls
    the turtle according the goal received from the
    client.
    """
    def __init__(self):
        # Initialize Simple Action Server
        self._sas = actionlib.SimpleActionServer('/action_turtle',
                                                 msgTurtleAction,
                                                 execute_cb=self.on_goal,
                                                 auto_start=False)

        self._config_ros_pub_topic = '/turtle1/cmd_vel'
        self._config_ros_sub_topic = '/turtle1/pose'

        # Subscribe to pose topic to get current x, y and theta
        self._handle_sub_pose = rospy.Subscriber(self._config_ros_sub_topic,
                                                 Pose, self.pose_callback)

        self._curr_x = 0
        self._curr_y = 0
        self._curr_theta = 0

        # Start the Action Server
        self._sas.start()
        rospy.loginfo("Started Turtle Simple Action Server.")

    def pose_callback(self, pose_message):
        """
        Callback Function for ROS Topic ('/turtle1/pose') Subscription
        """
        self._curr_x = pose_message.x
        self._curr_y = pose_message.y
        self._curr_theta = pose_message.theta

    def _move_straight(self, param_dis, param_speed, param_dir):
        """
        Function to move the turtle in turtlesim_node straight
        """
        obj_velocity_msg = Twist()

        # Store the start position of the turtle
        start_x = self._curr_x
        start_y = self._curr_y

        # Move the turtle till it reaches the desired
        # position by publishing to Velocity topic
        handle_pub_vel = rospy.Publisher(self._config_ros_pub_topic,
                                         Twist, queue_size=200)

        var_loop_rate = rospy.Rate(100)

        # Set the Speed of the Turtle according to the direction
        if param_dir == 'b':
            obj_velocity_msg.linear.x = (-1) * abs(int(param_speed))
        else:
            obj_velocity_msg.linear.x = abs(int(param_speed))

        # Move till desired distance is covered
        dis_moved = 0.0

        while not rospy.is_shutdown():

            # Send feedback to the client
            obj_msg_feedback = msgTurtleFeedback()

            obj_msg_feedback.cur_x = self._curr_x
            obj_msg_feedback.cur_y = self._curr_y
            obj_msg_feedback.cur_theta = self._curr_theta

            self._sas.publish_feedback(obj_msg_feedback)

            if dis_moved < param_dis:
                handle_pub_vel.publish(obj_velocity_msg)

                var_loop_rate.sleep()

                d_x = self._curr_x - start_x
                d_y = self._curr_y - start_y
                dis_moved = abs(math.sqrt((d_x ** 2) + (d_y ** 2)))
                rospy.logdebug('Distance Moved: {}'.format(dis_moved))
            else:
                break

        # Stop the Turtle after desired distance is covered
        obj_velocity_msg.linear.x = 0
        handle_pub_vel.publish(obj_velocity_msg)
        rospy.logdebug('Destination Reached')

    def _rotate(self, param_degree, param_speed, param_dir):
        """
        Rotate the turtle in turtlesim_node
        """
        obj_velocity_msg = Twist()

        # Store start Theta of the turtle
        start_degree = abs(math.degrees(self._curr_theta))
        current_degree = abs(math.degrees(self._curr_theta))

        # Rotate the turtle till desired angle is reached
        handle_pub_vel = rospy.Publisher(self._config_ros_pub_topic,
                                         Twist, queue_size=200)

        var_loop_rate = rospy.Rate(100)

        # Set the speed of rotation according to param_dir
        if param_dir == 'a':
            # Anticlockwise
            obj_velocity_msg.angular.z = math.radians(abs(int(param_speed)))
        else:
            # Clockwise
            obj_velocity_msg.angular.z = (-1) * \
                math.radians(abs(int(param_speed)))

        # Rotate till desired angle is reached
        degree_rotated = 0.0

        while not rospy.is_shutdown():
            if round(degree_rotated, 2) < param_degree:
                handle_pub_vel.publish(obj_velocity_msg)

                var_loop_rate.sleep()

                current_degree = abs(math.degrees(self._curr_theta))
                degree_rotated = abs(current_degree - start_degree)
                rospy.logdebug('Degree Rotated: {}'.format(degree_rotated))
            else:
                break

        # Stop the Turtle after the desired angle is reached
        obj_velocity_msg.angular.z = 0
        handle_pub_vel.publish(obj_velocity_msg)
        rospy.logdebug('Angle Reached')

    def on_goal(self, obj_msg_goal):
        """
        Process goals and send results
        """
        rospy.loginfo("Received a Goal from Client.")
        rospy.loginfo("Distance: " + str(obj_msg_goal.distance) + " Angle: " +
                      str(obj_msg_goal.angle))

        # --- Goal Processing Section ---
        self._rotate(obj_msg_goal.angle, 15, 'a')
        self._move_straight(obj_msg_goal.distance, 1, 'f')

        # Send Result to the Client
        obj_msg_result = msgTurtleResult()
        obj_msg_result.final_x = self._curr_x
        obj_msg_result.final_y = self._curr_y
        obj_msg_result.final_theta = self._curr_theta

        rospy.loginfo("send goal result to client")
        self._sas.set_succeeded(obj_msg_result)


def main():
    """
    Initializes the node and start the server
    """
    # Initialize ROS Node
    rospy.init_node('node_simple_action_server_turtle')

    # Create Simple Action Server object.
    obj_server = SimpleActionServerTurtle()

    # Do not exit and loop forever.
    rospy.spin()


if __name__ == '__main__':
    main()
