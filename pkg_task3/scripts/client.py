#!/usr/bin/env python2
import rospy
import actionlib

from pkg_task3.msg import ur5PickupAction
from pkg_task3.msg import ur5PickupGoal
from hrwros_gazebo.msg import LogicalCameraImage, Model
from models import debug
from ConveyorControl import Conveyor

processed = []

class CameraClient:
    def __init__(self):
        self.simple_client = actionlib.SimpleActionClient("/ur5_pickup", ur5PickupAction)

        self.conveyor = Conveyor()

        rospy.loginfo("CameraClient: Waiting for Simple Action Server")
        self.simple_client.wait_for_server()
        rospy.loginfo("CameraClient: Simple Action Server up")

        self.processing = False

        self.conveyor.set_power(55)

        # Subscribe to the topic where MQTT msgs are published.
        self.camera = rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                        LogicalCameraImage, self.process_frame,
                                        queue_size=1)
        


    def send_goal(self, name, pose):
        goal = ur5PickupGoal(package_name=name, package_pose=pose)

        self.simple_client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

        debug('Goal: "name = {}, pose = {}" sent'.format(name, str(pose)))

    def done_callback(self, status, result):
        # Callback for result
        # TODO
        debug("Client: received {} from server".format(result))
        self.processing = False
        if result.success == True:
            self.conveyor.set_power(55)
        else:
            debug("trying again")
            self.conveyor.set_power(20)
            rospy.sleep(1)
            self.conveyor.stop()

    def feedback_callback(self, feedback):
        # Callback for feedback
        rospy.logdebug("feedback: current arm pose {}".format(str(feedback.arm_pose)))

    def process_frame(self, msg):
        if self.processing:
            return

        models = msg.models
        if len(models) == 0 or (len(models) == 1 and models[0].type == "ur5"):
            debug('CameraClient: Callback on no models')
            return
        debug("CameraClient: some object in frame")

        current_packages = []
        for model in models:
            if model.type == "ur5":
                continue
            if model.type not in processed:
               current_packages.append(model) 
        if len(current_packages) == 0:
            debug('no new packages')
            return
        # TODO: Make sure the first package in the models list is the first
        # package on the conveyor
        pos_y = current_packages[0].pose.position.y
        debug("pos_y: {}".format(pos_y))
        if pos_y > -0.26 and pos_y <= 0.3:
            debug("CameraClient: object in pickable range, stopping conveyor")
            self.conveyor.stop()
            self.send_goal(current_packages[0].type,
                           current_packages[0].pose)
            self.processing = True
            debug("poping " + current_packages[0].type)
            processed.append(current_packages.pop(0).type)
            return
            
        self.conveyor.set_power(45)

def main():
    rospy.init_node("node_logical_camera_client")

    CameraClient()

    rospy.spin()


if __name__ == "__main__":
    main()