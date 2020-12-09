#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import Pose, PoseStamped

from pkg_task3.msg import ur5PickupAction
from pkg_task3.msg import ur5PickupResult
from pkg_task3.msg import ur5PickupFeedback
from models import debug, gazebo_red_bin, ur5_starting_pose
from Ur5Control import Ur5
from tf import TF

class SimpleActionServerTurtle:
    def __init__(self):
        # Initialize Simple Action Server
        self._sas = actionlib.SimpleActionServer("/ur5_pickup",
                                                 ur5PickupAction,
                                                 execute_cb=self.on_goal,
                                                 auto_start=False)
        # Start the Action Server
        self._ref_frame = "world"
        self._target_frame = "logical_camera_2_{}_frame"
        self._ur5 = Ur5()
        self._tf = TF()
        self._delta = 0.19

        self._ur5.go_to_pose(ur5_starting_pose)

        rospy.loginfo("Started Simple Action Server.")
        self._sas.start()

    def pose_callback(self, pose_message):
        self._curr_x = pose_message.x
        self._curr_y = pose_message.y
        self._curr_theta = pose_message.theta

    def on_goal(self, obj_msg_goal):
        """
        Process goals and send results
        """
        debug("Received a Goal from Client.")
        debug("package: " + str(obj_msg_goal.package_name) + " pose: " +
                      str(obj_msg_goal.package_pose))

        # Send Result to the Client
        obj_msg_result = ur5PickupResult()
        obj_msg_result.success = True

        model = obj_msg_goal.package_name

        trans = self._tf.lookup_transform(self._ref_frame, self._target_frame.format(model))

        package_pose = PoseStamped()
        package_pose.header.frame_id = "world"

        package_pose.pose.position.x = trans.transform.translation.x
        package_pose.pose.position.y = trans.transform.translation.y
        package_pose.pose.position.z = trans.transform.translation.z
        package_pose.pose.orientation.x = trans.transform.rotation.x
        package_pose.pose.orientation.y = trans.transform.rotation.y
        package_pose.pose.orientation.z = trans.transform.rotation.z
        package_pose.pose.orientation.w = trans.transform.rotation.w

        ur5_arm_pose = Pose()
        ur5_arm_pose.position.x = trans.transform.translation.x                                                                                                                       
        ur5_arm_pose.position.y = trans.transform.translation.y
        ur5_arm_pose.position.z = trans.transform.translation.z + self._delta
        ur5_arm_pose.orientation.x = -0.5
        ur5_arm_pose.orientation.y = -0.5
        ur5_arm_pose.orientation.z = 0.5
        ur5_arm_pose.orientation.w = 0.5

        debug('trying to pickup package')
        tries = 3
        while tries > 0 and not self._ur5.go_to_pose(ur5_arm_pose) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            tries -= 1
        if tries == 0:
            debug('trying failed')
            obj_msg_result.success = False
            self._sas.set_succeeded(obj_msg_result)
            return

        self._ur5.gripper(model, True)

        while not self._ur5.go_to_pose(gazebo_red_bin) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self._ur5.gripper(model, False)

        while not self._ur5.go_to_pose(ur5_starting_pose) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        
        debug("sending goal result to client")
        self._sas.set_succeeded(obj_msg_result)


def main():
    """
    Initializes the node and start the server
    """
    # Initialize ROS Node
    rospy.init_node('node_simple_action_server_turtle')

    # Create Simple Action Server object.
    SimpleActionServerTurtle()

    # Do not exit and loop forever.
    rospy.spin()


if __name__ == '__main__':
    main()
