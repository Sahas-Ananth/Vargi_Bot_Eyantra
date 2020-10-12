#!/usr/bin/env python

"""
Ros node to make to the turtle draw a circle
"""
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleController(object):
    """
    Controls the turtle to draw a circle
    """
    def __init__(self):
        # initial turtle coordinates
        self.initial_coords = [0, 0]

        # turtle current position
        self.current_coords = [0, 0]

        self.initial_coords_updated = False

        # turtle angular speed (rad/s)
        self.turtle_speed = 0.5*math.pi
        # radius of the circle to be drawn (0 < r < 2.5)
        self.circle_radius = 1.5

        rospy.init_node("node_turtle_revolve", anonymous=False)

        # Subscriber for "turtle1/pose" to get current location and speed.
        rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)

        # Publisher for "turtle1/cmd_vel" to publish angular and linear vel.
        self.velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist,
                                            queue_size=1000)

    def update_pose(self, msg):
        """
        Callback funtion for topic "/turtle1/pose".
        Updates x, y and theta.
        """
        self.current_coords = [msg.x, msg.y]

        if not self.initial_coords_updated:
            self.initial_coords = [msg.x, msg.y]
            self.initial_coords_updated = True

    def run(self):
        """
        Runs the controller to make a circle of specified radius
        and displays statistics like turtle location and offset error
        """
        msg = Twist()

        msg.linear.x = self.turtle_speed * self.circle_radius
        msg.angular.z = self.turtle_speed

        t_0 = rospy.Time.now().to_sec()
        total_time = 2 * math.pi / self.turtle_speed

        rospy.loginfo("Moving in circle")

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.velocity_pub.publish(msg)
            t_1 = rospy.Time.now().to_sec()

            rate.sleep()

            if abs(total_time - (t_1 - t_0)) < 0.05:
                break

        msg.linear.x = 0
        msg.angular.z = 0
        self.velocity_pub.publish(msg)

        msg.linear.x = 0
        msg.angular.z = 3.14 / 2
        self.velocity_pub.publish(msg)

        # Displaying the statistics
        rospy.loginfo('intial pos: {%0.3f, %0.3f} final pos: {%0.3f, %0.3f}'
                      ' offset error: %0.3f', self.initial_coords[0],
                      self.initial_coords[1], self.current_coords[0],
                      self.current_coords[1], self._offset_error())

        rospy.loginfo("Goal reached")

    def _offset_error(self):
        """
        Calculates the offset error between the initial and
        final coordinates
        """
        return math.sqrt((self.initial_coords[0]-self.current_coords[0])**2 +
                         (self.initial_coords[1]-self.current_coords[1])**2)


if __name__ == "__main__":
    try:
        CONTROLLER = TurtleController()
        CONTROLLER.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
