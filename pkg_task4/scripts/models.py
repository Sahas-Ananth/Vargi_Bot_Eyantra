#! /usr/bin/env python

"""
Defines all the poses for the objects and the joint angles
for the arm.
"""

import geometry_msgs.msg

ur5_starting_angles = [0.1026, -139.13, -39.04, -91.14, 90.005, 179.35]

# ur5_new_starting_angles = [0.711, -141.987, -49.982, -81.277, 89.623, 179.99]
ur5_new_starting_angles = [0.706232802096, -141.981689307, -
                           49.9858133044, -81.2825158746, 89.6194706419, 179.984754632]

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

packagen00_angles = [-56.8377290053, -66.991454777, -
                     0.782286913554, -111.907937244, -117.390233662, 0.00382732069232]
packagen01_angles = [118.267968731, -118.301705137,
                     32.6628430494, -96.1926169566, 67.3959495859, -177.201452159]
packagen02_angles = [55.8426592228, -113.759379846,
                     2.89923281727, -68.4565933187, 121.167123356, -163.107709077]

packagen10_angles = [-53.2580995235, -96.6104012757,
                     81.06233691, -164.005477866, -126.002684119, -86.4607748888]
packagen11_angles = [-122.734825794, -118.044029398,
                     96.6353275631, -156.500303241, -56.4404181573, -87.8650948306]
packagen12_angles = [54.693184432, -96.5173266697, -
                     40.4036790652, 137.026011422, -126.030014075, 93.3026112139]

packagen20_angles = [-59.3337761701, -99.9485203319,
                     90.8040200235, 11.2727008002, 116.552409852, -2.20988577574]
packagen21_angles = [125.532011493, -59.7151234418, -
                     129.594446118, 9.12204955794, 55.196133377, -86.6265769885]
packagen22_angles = [-162.349819568, -97.0726608053,
                     86.4450582827, 10.6615728772, 21.1960115017, -0.0121865407722]

packagen30_angles = [-55.5848257648, -92.3689940625,
                     119.814000153, -26.0948490068, 122.395881177, 92.2313432542]
packagen31_angles = [-120.79769795, -117.115246557,
                     135.794306272, -18.6786948062, 56.720354748, -0.00700065583683]
packagen32_angles = [52.5776560284, -112.475159569, -
                     130.41174184, 62.716838271, 128.132401393, -86.8736397541]


# pkg_colours = {"packagen00": "red",
#                "packagen01": "green",
#                "packagen02": "yellow",

#                "packagen10": "green",
#                "packagen11": "yellow",
#                "packagen12": "red",

#                "packagen20": "yellow",
#                "packagen21": "red",
#                "packagen22": "green",

#                "packagen30": "red",
#                "packagen31": "yellow",
#                "packagen32": "green"
#                }


def debug(f):
    print (u'\u001b[36;1m' + f + u'\u001b[0m')
