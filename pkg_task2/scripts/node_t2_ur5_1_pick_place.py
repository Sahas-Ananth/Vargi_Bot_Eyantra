#! /usr/bin/env python

from os import fstatvfs
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from pkg_vb_sim.srv import *


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/self._group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = 'package'

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planningscene Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self._box_name
        scene = self._scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, timeout=4):
        box_name = self._box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"

        box_pose.pose.position.x = 0.04
        box_pose.pose.position.y = 0.44
        box_pose.pose.position.z = 1.88

        box_pose.pose.orientation.w = 1.0
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = "vacuum_gripper_link"
        group_names = self._group_names

        grasping_group = "ur5_1_planning_group"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo("Attached package")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):

        box_name = self._box_name
        scene = self._scene
        eef_link = "vacuum_gripper_link"  # self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        rospy.loginfo("Dettached package")
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):

        box_name = self._box_name
        scene = self._scene

        scene.remove_world_object(box_name)

        # **Note:** The object must be detached before we can remove it from the world

        rospy.loginfo("Removed package")
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    ur5.add_box()

    gripper = rospy.ServiceProxy(
        name="/eyrc/vb/ur5_1/activate_vacuum_gripper", service_class=vacuumGripper)

    rospy.wait_for_service("/eyrc/vb/ur5_1/activate_vacuum_gripper")

    ur5_get2box = geometry_msgs.msg.Pose()
    ur5_get2box.position.x = 0.0205250112852
    ur5_get2box.position.y = 0.241005180661
    ur5_get2box.position.z = 1.92153028989
    ur5_get2box.orientation.x = 2.99942395011e-05
    ur5_get2box.orientation.y = 0.70704738914
    ur5_get2box.orientation.z = -6.64621547352e-05
    ur5_get2box.orientation.w = 0.707166164485

    ur5_back_away1 = geometry_msgs.msg.Pose()
    ur5_back_away1.position.x = 0.0206967548413
    ur5_back_away1.position.y = 0.114642749137
    ur5_back_away1.position.z = 1.92160160525
    ur5_back_away1.orientation.x = 0.14414193291
    ur5_back_away1.orientation.y = 0.692579908475
    ur5_back_away1.orientation.z = 0.143963920595
    ur5_back_away1.orientation.w = 0.691975839984

    ur5_back_away2 = geometry_msgs.msg.Pose()
    ur5_back_away2.position.x = 0.10724661214
    ur5_back_away2.position.y = -0.060267853482
    ur5_back_away2.position.z = 1.93844623103
    ur5_back_away2.orientation.x = 0.143148307905
    ur5_back_away2.orientation.y = 0.692629892027
    ur5_back_away2.orientation.z = 0.143113841985
    ur5_back_away2.orientation.w = 0.69230832932

    ur5_back_away3 = geometry_msgs.msg.Pose()
    ur5_back_away3.position.x = 0.105759985315
    ur5_back_away3.position.y = -0.489736470138
    ur5_back_away3.position.z = 1.78029990242
    ur5_back_away3.orientation.x = 0.143212005986
    ur5_back_away3.orientation.y = 0.692642620997
    ur5_back_away3.orientation.z = 0.143080417953
    ur5_back_away3.orientation.w = 0.692289328907

    # ur5_got2bin = geometry_msgs.msg.Pose()
    # ur5_got2bin.position.x = -0.768924999851
    # ur5_got2bin.position.y = -0.372580965627
    # ur5_got2bin.position.z = 1.2282796422
    # ur5_got2bin.orientation.x = -0.494160491129
    # ur5_got2bin.orientation.y = 0.352967401773
    # ur5_got2bin.orientation.z = -0.207456091249
    # ur5_got2bin.orientation.w = 0.766929848485

    ur5_bin = geometry_msgs.msg.Pose()
    ur5_bin.position.x = -0.744829906687
    ur5_bin.position.y = -0.14353672654
    ur5_bin.position.z = 1.197921419
    ur5_bin.orientation.x = -0.706796676311
    ur5_bin.orientation.y = 0.34389160369
    ur5_bin.orientation.z = -0.245423861274
    ur5_bin.orientation.w = 0.567401226281

    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose("allZeros")
        rospy.sleep(0.5)
        ur5.go_to_pose(ur5_get2box)
        rospy.sleep(0.5)
        ur5.attach_box(1)
        gripper(True)
        rospy.sleep(0.5)
        ur5.go_to_pose(ur5_back_away1)
        rospy.sleep(0.5)
        ur5.go_to_pose(ur5_back_away2)
        rospy.sleep(0.5)
        ur5.go_to_pose(ur5_back_away3)
        rospy.sleep(0.5)
        # ur5.go_to_pose(ur5_got2bin)
        # rospy.sleep(0.5)
        ur5.go_to_pose(ur5_bin)
        rospy.sleep(0.5)
        ur5.detach_box(1)
        gripper(False)
        rospy.sleep(0.5)
        ur5.go_to_predefined_pose("allZeros")
        ur5.remove_box(2)
        rospy.sleep(2)
        break

    del ur5


if __name__ == '__main__':
    main()
