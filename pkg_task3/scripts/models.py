#! /usr/bin/env python
import geometry_msgs.msg

ur5_starting_angles = [0.1026, -139.13, -39.04, -91.14, 90.005, 179.35]

gazebo_red_bin_angles = [-95.35, -105.78, -82.37, -80.629, 86.149, 77.032]

gazebo_green_bin_angles = [-171.77, -114.084, -69.485, -82.30, 92.412, -7.447]

gazebo_blue_bin_angles = [-89.38, -69.146, 63.701, -89.01, -89.97, 89.736]

# UR5 Starting position
ur5_starting_pose = geometry_msgs.msg.Pose()
ur5_starting_pose.position.x = -0.80
ur5_starting_pose.position.y = 0.10
ur5_starting_pose.position.z = 1.25
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