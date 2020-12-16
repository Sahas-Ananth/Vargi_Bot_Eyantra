#!/usr/bin/env python2

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
This module represents the logical camera node. And is responsible
for detecting packages through the logical camera and sending
action goals to the UR5 controller.
"""

import rospy
import actionlib

from pkg_task3.msg import ur5PickupAction
from pkg_task3.msg import ur5PickupGoal
from hrwros_gazebo.msg import LogicalCameraImage
from ConveyorControl import Conveyor

class Camera(object):
    """
    Camera class is responsible for detecting the packages through the logical
    camera and sends goals to the UR5 controller on the topic /ur5_pickup.
    For this to be possible a user defined action file is generated *ur5PickUpAction*.
    """
    def __init__(self):
        self.simple_client = actionlib.SimpleActionClient("/ur5_pickup", ur5PickupAction)

        self.conveyor = Conveyor()

        self.processing = False
        self.processed = []
        self.current_package = ""

        rospy.loginfo('\033[94;1m' + "Camera: Waiting for Simple Action Server" + '\033[0m')
        self.simple_client.wait_for_server()
        rospy.loginfo('\033[34;1m' + "Camera: Simple Action Server up." + '\033[0m')

        # Start the conveyor
        self.conveyor.set_power(70)

        self.camera = rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                       LogicalCameraImage, self.process_frame,
                                       queue_size=1)

    def send_goal(self, name, pose):
        """
            Sends the goal to the UR5 controller on detecting
            a package in pickable range.
            NOTE: Currently only the package_name is used by
            the controller. But for future proof we also send
            the pose.
        """
        goal = ur5PickupGoal(package_name=name, package_pose=pose)

        self.simple_client.send_goal(goal, done_cb=self._done_callback,
                                     feedback_cb=self._feedback_callback)
        rospy.loginfo('\033[34;1m' +
                      "Goal: name = {}\npose = {} sent.".format(name, str(pose)) + '\033[0m')

    def _done_callback(self, _status, result):
        self.processing = False

        if result.success is True:
            self.processed.append(self.current_package)
            if len(self.processed) != 3:
                self.conveyor.set_power(70)
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
        if self.processing:
            return

        if len(self.processed) == 3:
            self.conveyor.stop()

        models = msg.models
        if not models or (len(models) == 1 and models[0].type == "ur5"):
            return

        current_packages = []
        for model in models:
            if model.type == "ur5":
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
                           current_packages[0].pose)
            self.current_package = current_packages[0].type
            self.processing = True
            return


def main():
    """
        Instantiates a rospy node and Camera object.
    """
    rospy.init_node("node_t3_camera_control")

    Camera()

    rospy.spin()

if __name__ == "__main__":
    main()
