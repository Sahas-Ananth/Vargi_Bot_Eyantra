#! /usr/bin/env python

# Copyright <2020> <Niteesh G S>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Defines all the poses for the objects and the joint angles
for the arm.
"""

import geometry_msgs.msg

ur5_starting_angles = [0.1026, -139.13, -39.04, -91.14, 90.005, 179.35]

ur5_new_starting_angles = [0.711, -141.987, -49.982, -81.277, 89.623, 179.99]

ur5_green_box_angles = [-0.0427, -121.54, -85, -62.77, 90.012, 179.215]

ur5_blue_box_angles = [-4.057, -141.290, -37.388, -90.47, 90.255, 171.178]

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