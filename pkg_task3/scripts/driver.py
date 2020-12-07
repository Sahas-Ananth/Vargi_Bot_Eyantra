#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
from pkg_vb_sim.srv import *
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates


class Ur5Moveit:

    # Constructor
    def __init__(self):

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/self._group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        # self._box_name = "packagen1"

        self.gripper = rospy.ServiceProxy(
            name="/eyrc/vb/ur5_1/activate_vacuum_gripper", service_class=vacuumGripper)
        self.set_conveyer_speed = rospy.ServiceProxy(
            name="/eyrc/vb/conveyor/set_power", service_class=conveyorBeltPowerMsg)

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planningscene Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # """def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self._box_name
        scene = self._scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # return False"""

    def add_box(self, boxpose, boxName, timeout=4):
        box_name = boxName
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()

        box_pose.pose.position.x = boxpose.position.x
        box_pose.pose.position.y = boxpose.position.y
        box_pose.pose.position.z = boxpose.position.z
        box_pose.pose.orientation.x = boxpose.orientation.x
        box_pose.pose.orientation.y = boxpose.orientation.y
        box_pose.pose.orientation.z = boxpose.orientation.z
        box_pose.pose.orientation.x = boxpose.orientation.w
        box_pose.header.frame_id = "world"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        # self.box_name = box_name
        # return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, boxName, timeout=4):
        box_name = boxName
        robot = self._robot
        scene = self._scene
        eef_link = "vacuum_gripper_link"
        group_names = self._group_names

        grasping_group = "ur5_1_planning_group"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo("Attached package")

        # return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, boxName, timeout=4):

        box_name = boxName
        scene = self._scene
        eef_link = "vacuum_gripper_link"  # self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        rospy.loginfo("Dettached package")
        # return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, boxName, timeout=4):

        box_name = boxName
        scene = self._scene

        scene.remove_world_object(box_name)

        # **Note:** The object must be detached before we can remove it from the world

        rospy.loginfo("Removed package " + boxName)
        # return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def get_transform(self, ref_frame, target_frame):
        try:
            trans = self._tfBuffer.lookup_transform(
                ref_frame, target_frame, rospy.Time())

            rospy.loginfo("\nTransform of {} with respect to {}\n".format(target_frame, ref_frame) +
                          "Translation: \n" +
                          "x: {} \n".format(trans.transform.translation.x) +
                          "y: {} \n".format(trans.transform.translation.y) +
                          "z: {} \n".format(trans.transform.translation.z) +
                          "\n" +
                          "Orientation: \n" +
                          "x: {} \n".format(trans.transform.rotation.x) +
                          "y: {} \n".format(trans.transform.rotation.y) +
                          "z: {} \n".format(trans.transform.rotation.z) +
                          "w: {} \n".format(trans.transform.rotation.w))
            return trans

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

###################################################################################################################################


class GazeboModel(object):
    def __init__(self, gazebo_models_list):

        # We wait for the topic to be available and when it is then retrieve the index of each model
        # This was separated from callbal to avoid doing this in each callback
        self._robots_models_dict = {}
        self._robots_pose_list = []
        self._robots_index_dict = {}
        self._gazebo_models_list = gazebo_models_list

        self.get_robot_index()

        # We now start the suscriber once we have the indexes of each model
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def get_robot_index(self):

        data = None
        found_all_models_names = False
        while not found_all_models_names and not rospy.is_shutdown():
            rospy.loginfo("Retrieveing Model indexes ")
            try:
                data = rospy.wait_for_message(
                    "/gazebo/model_states", ModelStates, timeout=5)
                # Save it in the format {"robot1":4,"robot2":2}
                if data:
                    # Here we have model_states data, but not guarantee that the models we want are there
                    not_found_models_list = []
                    for robot_name in self._gazebo_models_list:
                        robot_name_found = self.update_robot_index(
                            data, robot_name)
                        if not robot_name_found:
                            not_found_models_list.append(robot_name)
                            break
                    found_all_models_names = len(
                        self._robots_index_dict) == len(self._gazebo_models_list)
                else:
                    rospy.loginfo(
                        "Topic /gazebo/model_states NOT Ready yet, trying again ")

            except Exception as e:
                s = str(e)
                rospy.loginfo("Error in get_robot_index = " + s)

        assert found_all_models_names, "Models Missing==" + \
            str(not_found_models_list)
        rospy.loginfo("Final robots_index_dict =  %s ",
                      str(self._robots_index_dict))

    def update_robot_index(self, data, robot_name):
        try:
            index = data.name.index(robot_name)
            self._robots_index_dict[robot_name] = index
            found = True
        except ValueError:
            rospy.loginfo("Robot Name=" + str(robot_name) +
                          ", is NOT in model_state, trying again")
            found = False

        return found

    def callback(self, data):

        for robot_name in self._gazebo_models_list:
            # Retrieve the corresponding index
            robot_name_found = self.update_robot_index(data, robot_name)
            if robot_name_found:
                data_index = self._robots_index_dict[robot_name]
                # Get the pose data from theat index
                try:
                    data_pose = data.pose[data_index]
                except IndexError:
                    rospy.logwarn("The model with data index " +
                                  str(data_index) + ", something went wrong.")
                    data_pose = None
            else:
                data_pose = None
            # Save the pose inside the dict {"robot1":pose1,"robot2":pose2}
            self._robots_models_dict[robot_name] = data_pose

    def get_model_pose(self, robot_name):

        pose_now = None

        try:
            pose_now = self._robots_models_dict[robot_name]
        except Exception as e:
            s = str(e)
            rospy.loginfo("Error, The _robots_models_dict is not ready = " + s)

        return pose_now
