#!/usr/bin/env python

import rospy
import actionlib

from pkg_task4.msg import task4Action
from pkg_task4.msg import task4Goal
from hrwros_gazebo.msg import LogicalCameraImage
from ConveyorControl import Conveyor
from models import *


class Camera(object):
    """
    Camera class is responsible for detecting the packages through the logical
    camera and sends goals to the UR5 controller on the topic /ur5_pickup.
    For this to be possible a user defined action file is generated *task4Action*.
    """

    def __init__(self):
        self.simple_client = actionlib.SimpleActionClient(
            "/ur5_pkg_sorter", task4Action)

        self.conveyor = Conveyor()

        # self.camera2d = Camera1()

        # self.package_colours = self.camera2d.packages
        self.package_colours = pkg_colours

        rospy.loginfo("Colour of packages: {}\n".format(self.package_colours))

        self.processing = False
        self.processed = []
        self.current_package = ""

        rospy.loginfo(
            '\033[94;1m' + "Camera: Waiting for Simple Action Server" + '\033[0m')
        self.simple_client.wait_for_server()
        rospy.loginfo(
            '\033[34;1m' + "Camera: Simple Action Server up." + '\033[0m')

        # Start the conveyor
        self.conveyor.set_power(100)

        self.camera = rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                       LogicalCameraImage, self.process_frame,
                                       queue_size=1)

    def send_goal(self, name, colour, package_pose, robot_pose):
        """
            Sends the goal to the UR5 controller on detecting
            a package in pickable range.
            NOTE: Currently only the package_name is used by
            the controller. But for future proof we also send
            the pose.
        """
        goal = task4Goal(package_name=name, colour=colour,
                         package_pose=package_pose, ur5_pose=robot_pose)

        self.simple_client.send_goal(goal, done_cb=self._done_callback,
                                     feedback_cb=self._feedback_callback)
        rospy.loginfo('\033[34;1m' +
                      "Goal: name = {}\nColour: {}\nPackage_Pose = {}\nRobot_pose = {} sent.".format(name, colour, str(package_pose), str(robot_pose)) + '\033[0m')

    def _done_callback(self, _status, result):
        self.processing = False

        if result.success is True:
            self.processed.append(self.current_package)
            if len(self.processed) != 9:
                self.conveyor.set_power(100)
        else:
            self.conveyor.set_power(25)
            rospy.sleep(1)
            self.conveyor.stop()

    def _feedback_callback(self, feedback):
        pass

    def process_frame(self, msg):
        """
            process_frame: Is responsible for the detecting the
            package that is in pickable range and sending the goal
            to the ur5 controller.
        """
        last_ur5_pose = []
        if self.processing:
            return

        if len(self.processed) == 9:
            self.conveyor.stop()

        models = msg.models
        if not models or (len(models) == 1 and models[0].type == "ur5"):
            return

        current_packages = []
        for model in models:
            if model.type == "ur5":
                last_ur5_pose.append(model)
                continue
            if model.type not in self.processed:
                current_packages.append(model)

        if not current_packages:
            return

        # NOTE: Make sure the first package in the models list is the first
        # package on the conveyor
        pos_y = current_packages[0].pose.position.y

        if pos_y >= 0.10 and pos_y <= 0.20:
            self.conveyor.stop()
            self.send_goal(current_packages[0].type,
                           self.package_colours[current_packages[0].type],
                           current_packages[0].pose, last_ur5_pose[0].pose)
            self.current_package = current_packages[0].type
            self.processing = True
            return


def main():
    """
        Instantiates a rospy node and Camera object.
    """
    rospy.init_node("Logical_camera2_control")

    Camera()

    rospy.spin()


if __name__ == "__main__":
    main()
