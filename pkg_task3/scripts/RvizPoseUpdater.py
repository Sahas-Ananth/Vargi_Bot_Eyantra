#!/usr/bin/env python
import rospy

from driver import GazeboModel, Ur5Moveit

rospy.init_node('Box_Rviz_Pose_Updater', anonymous=True)
gazebo_models_list = ['packagen1', 'packagen2', 'packagen3']
gz_model = GazeboModel(gazebo_models_list)
environment = Ur5Moveit()
rate = rospy.Rate(60)
try:
    while not rospy.is_shutdown():
        environment.add_box(boxpose=gz_model.get_model_pose(
            gazebo_models_list[0]), boxName=gazebo_models_list[0])
        environment.add_box(boxpose=gz_model.get_model_pose(
            gazebo_models_list[1]), boxName=gazebo_models_list[1])
        environment.add_box(boxpose=gz_model.get_model_pose(
            gazebo_models_list[2]), boxName=gazebo_models_list[2])
except rospy.ROSInterruptException:
    rospy.loginfo("Closing...")
