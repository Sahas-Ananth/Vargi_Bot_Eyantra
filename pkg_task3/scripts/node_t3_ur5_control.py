#! /usr/bin/env python

from hrwros_gazebo.msg import LogicalCameraImage, Model
from ConveyorControl import Conveyor
from Ur5Control import Ur5
from tf import TF
from models import *

import rospy
import tf2_ros

class Task3:
    def __init__(self):
        rospy.init_node("node_t3_ur5_control")

        # Reference frames used in TF
        self._ref_frame = "world"
        self._target_frame = "logical_camera_2_{}_frame"

        # Offset to be maintained from the package to properly
        # pickup the object
        self._delta = vacuum_gripper_width + (box_length/2)

        # UR5 control instance
        self._ur5 = Ur5()
        # TF instance
        self._tf = TF()
        # Instance to control the conveyor belt
        self._conveyor = Conveyor()

        self._packages_processed = []
        self._packages_processing = []

        # Move the arm to a proper starting position
        while not self._ur5.go_to_pose(ur5_starting_pose) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.sleep(2)

        # Start the conveyor
        self._conveyor.set_power(60)

        # Subscribing to logical camera topic
        self._camera = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self._camera_callback, queue_size=1)

    def _camera_callback(self, msg):
        self._camera.unregister()

        models = msg.models

        if len(models) < 1:
            debug('callback early return len(models) < 1')
            return
        if len(models) == 1 and models[0].type == "ur5":
            debug('callback early return len(models) == 1')
            return

        for model in models:
            if model.type in self._packages_processed:
                return

        debug(str(models[0].pose))
        pos_y = models[0].pose.position.y
        if pos_y > 0.40:
            return

        self._conveyor.stop()

        for model in models:
            if model.type == "ur5":
                continue
            package = model.type
            if package not in self._packages_processing and \
               package not in self._packages_processed:

                debug('appending ' + str(package) + ' gazebo pacakges')
                self._packages_processing.append(package)
        self.execute()
        debug('exiting callback')

    def _get_bin(self, package):
        if package == "packagen1":
            return gazebo_red_bin
        elif package == "packagen2":
            return gazebo_green_bin
        elif package == "packagen3":
            return gazebo_blue_bin

    def run(self):
        debug('in run')

        if len(self._packages_processing) == 0:
            return True
        
        debug('package found, picking up pacakge')
        model = self._packages_processing[0]

        debug('-----------------------------------------')
        debug(self._ref_frame)
        debug(self._target_frame.format(model))
        debug('-----------------------------------------')

        trans = self._tf.lookup_transform(self._ref_frame, self._target_frame.format(model))

        package_pose = geometry_msgs.msg.PoseStamped()
        package_pose.header.frame_id = "world"

        package_pose.pose.position.x = trans.transform.translation.x
        package_pose.pose.position.y = trans.transform.translation.y
        package_pose.pose.position.z = trans.transform.translation.z
        package_pose.pose.orientation.x = trans.transform.rotation.x
        package_pose.pose.orientation.y = trans.transform.rotation.y
        package_pose.pose.orientation.z = trans.transform.rotation.z
        package_pose.pose.orientation.w = trans.transform.rotation.w

        ur5_arm_pose = geometry_msgs.msg.Pose()
        ur5_arm_pose.position.x = trans.transform.translation.x                                                                                                                       
        ur5_arm_pose.position.y = trans.transform.translation.y
        ur5_arm_pose.position.z = trans.transform.translation.z + self._delta
        ur5_arm_pose.orientation.x = -0.5
        ur5_arm_pose.orientation.y = -0.5
        ur5_arm_pose.orientation.z = 0.5
        ur5_arm_pose.orientation.w = 0.5

        # --------------------'
        # print ur5_arm_pose
        # --------------------'

        # self._ur5.add_box(model, package_pose, box_length)

        debug('trying to pickup package')
        tries = 3
        while tries > 0 and not self._ur5.go_to_pose(ur5_arm_pose) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            tries -= 1
        if tries == 0:
            debug('trying failed')
            return False

        self._ur5.gripper(model, True)

        while not self._ur5.go_to_pose(self._get_bin(model)) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self._ur5.gripper(model, False)

        debug('removing package from gazebo package list')
        self._packages_processed.append(model)
        self._packages_processing.pop(0)

        while not self._ur5.go_to_pose(ur5_starting_pose) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        
        debug('in run')
        return True
    
    def execute(self):
        debug('in execute')
        debug(str(self._packages_processing))
        for _ in self._packages_processing:
            pickedup = self.run()
            while not pickedup:
                self._conveyor.set_power(40)
                rospy.sleep(1)
                self._conveyor.stop()
                pickedup = self.run()
        debug('exiting execute')
        self._conveyor.set_power(40)
        self._camera = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self._camera_callback)

if __name__ == '__main__':
    task3_solution = Task3()

    while not rospy.is_shutdown():
        task3_solution.execute()
        rospy.spin()