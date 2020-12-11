#!/usr/bin/env python2

import rospy
import actionlib

from pkg_task3.msg import ur5PickupAction
from pkg_task3.msg import ur5PickupGoal
from hrwros_gazebo.msg import LogicalCameraImage, Model
from ConveyorControl import Conveyor

class Camera:
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
        goal = ur5PickupGoal(package_name=name, package_pose=pose)

        self.simple_client.send_goal(goal, done_cb=self._done_callback, feedback_cb=self._feedback_callback)
        rospy.loginfo('\033[34;1m' + "Goal: name = {}\npose = {} sent.".format(name, str(pose)) + '\033[0m')

    def _done_callback(self, status, result):
        self.processing = False

        if result.success == True:
            self.processed.append(self.current_package)
            self.conveyor.set_power(70)
        else:
            self.conveyor.set_power(25)
            rospy.sleep(1)
            self.conveyor.stop()

    def _feedback_callback(self, feedback):
        pass

    def process_frame(self, msg):
        if self.processing:
            return

        if len(self.processed) == 3:
            self.conveyor.stop()

        models = msg.models
        if len(models) == 0 or (len(models) == 1 and models[0].type == "ur5"):
            return

        current_packages = []
        for model in models:
            if model.type == "ur5":
                continue
            if model.type not in self.processed:
               current_packages.append(model) 

        if len(current_packages) == 0:
            return

        # TODO: Make sure the first package in the models list is the first
        # package on the conveyor
        pos_y = current_packages[0].pose.position.y

        if pos_y >= 0.10 and pos_y <= 0.20:
            self.conveyor.stop()
            self.send_goal(current_packages[0].type,
                           current_packages[0].pose)
            self.current_package = current_packages[0].type
            self.processing = True
            return
            
        self.conveyor.set_power(70)

def main():
    rospy.init_node("node_t3_camera_control")

    Camera()

    rospy.spin()

if __name__ == "__main__":
    main()