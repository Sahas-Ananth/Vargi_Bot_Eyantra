#! /usr/bin/env python
import geometry_msgs.msg

# UR5 Starting position
ur5_starting_pose = geometry_msgs.msg.Pose()
ur5_starting_pose.position.x = -0.40
ur5_starting_pose.position.y = 0.107
ur5_starting_pose.position.z = 1.45
ur5_starting_pose.orientation.x = -0.5
ur5_starting_pose.orientation.y = -0.5
ur5_starting_pose.orientation.z = 0.5
ur5_starting_pose.orientation.w = 0.5

# Sorting bin location wrt world in gazebo
gazebo_red_bin = geometry_msgs.msg.Pose()
gazebo_red_bin.position.x = 0.11
gazebo_red_bin.position.y = 0.45
gazebo_red_bin.position.z = 1.30
gazebo_red_bin.orientation.x = 0
gazebo_red_bin.orientation.y = 0
gazebo_red_bin.orientation.z = 0
gazebo_red_bin.orientation.w = 0

gazebo_green_bin = geometry_msgs.msg.Pose()
gazebo_green_bin.position.x = 0.70
gazebo_green_bin.position.y = 0.03
gazebo_green_bin.position.z = 1.30
gazebo_green_bin.orientation.x = -0.5
gazebo_green_bin.orientation.y = -0.5
gazebo_green_bin.orientation.z = 0.5
gazebo_green_bin.orientation.w = 0.5

gazebo_blue_bin = geometry_msgs.msg.Pose()
gazebo_blue_bin.position.x = 0.04
gazebo_blue_bin.position.y = -0.65
gazebo_blue_bin.position.z = 1.30
gazebo_blue_bin.orientation.x = -0.5
gazebo_blue_bin.orientation.y = -0.5
gazebo_blue_bin.orientation.z = 0.5
gazebo_blue_bin.orientation.w = 0.5

box_length = 0.15               # Length of the Package
vacuum_gripper_width = 0.115    # Vacuum Gripper Width

def debug(f):
    print u'\u001b[36;1m' + f + u'\u001b[0m'