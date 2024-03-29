#! /usr/bin/env python

"""
This module represent an UR5 Arm and help in
manipulation.
Most of the code here is referenced from E-Yantra resources.
"""

import copy
import sys
import math
import yaml
import os
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

from pkg_vb_sim.srv import vacuumGripper
from std_srvs.srv import Empty


class Ur5Controller(object):

    # Constructor
    def __init__(self, arg_robot_name):

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)

        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._group.set_planner_id("RRT")
        self._group.set_planning_time(99)

        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ""

        self._gripper_service_name = "/eyrc/vb/ur5/activate_vacuum_gripper/{}".format(
            arg_robot_name)

        rospy.wait_for_service(self._gripper_service_name)
        self._gripper = rospy.ServiceProxy(
            name=self._gripper_service_name, service_class=vacuumGripper)

        rospy.loginfo(
            '\033[94m' + "{} Gripper active".format(arg_robot_name) + '\033[0m')

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ""

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def get_current_pose(self):
        return self._group.get_current_pose().pose

    def go_to_pose(self, arg_pose):
        rospy.loginfo(
            '\033[94m' + 'Moving to pose {}'.format(arg_pose) + '\033[0m')

        self._group.set_pose_target(arg_pose)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        if flag_plan is True:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name=""):
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

    def add_box(self, box_name, box_pose, box_size, timeout=4):
        scene = self._scene
        size = (box_size, box_size, box_size)
        scene.add_box(box_name, box_pose, size=size)

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name=box_name)

    def attach_box(self, box_name, timeout=4):
        robot = self._robot
        scene = self._scene
        eef_link = "vacuum_gripper_link"

        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo("Attached package")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout, box_name=box_name)

    def detach_box(self, box_name, timeout=4):
        scene = self._scene
        eef_link = "vacuum_gripper_link"  # self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        rospy.loginfo("Dettached package")
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout, box_name=box_name)

    def remove_box(self, box_name, timeout=4):
        scene = self._scene

        scene.remove_world_object(box_name)

        #! **Note:** The object must be detached before we can remove it from the world

        rospy.loginfo("Removed package")
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout, box_name=box_name)

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        rospy.loginfo(
            '\033[94m' + "Set joint angles to {}".format(arg_list_joint_angles) + '\033[0m')
        arg_list_joint_angles = map(math.radians, arg_list_joint_angles)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        self._group.stop()
        self._group.clear_pose_targets()

        if flag_plan is True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()
        return flag_success

    def save_prev_path(self, arg_file_path, file_name):
        file_path = arg_file_path + file_name + ".yaml"
        with open(file_path, 'w') as file_save:
            yaml.dump(self._computed_plan, file_save, default_flow_style=True)

        rospy.loginfo("File saved at: {}".format(file_path))

    def play_saved_path(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name + ".yaml"

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def hard_play_saved_path(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.play_saved_path(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

        return flag_success

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        self._group.set_named_target(arg_pose_name)
        self._computed_plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = self._computed_plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (self._computed_plan, _) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(self._computed_plan.joint_trajectory.points)
        if num_pts >= 3:
            del self._computed_plan.joint_trajectory.points[0]
            del self._computed_plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        return self._group.execute(self._computed_plan)

    def gripper(self, box_name, val):
        if val is True:
            self.attach_box(box_name, 0.5)
        else:
            self.detach_box(box_name, 0.5)
        self._gripper(val)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
