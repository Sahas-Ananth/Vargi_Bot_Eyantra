#! /usr/bin/env python

import rospy
from pkg_vb_sim.srv import conveyorBeltPowerMsg


class Conveyor(object):
    """
        This class is responsible for controlling conveyor.
        To use this, create an instance of this class and
        use the set_power and stop functions to control the
        conveyor without the need to manually issue service
        requests.
    """

    def __init__(self):
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        self._conveyor = rospy.ServiceProxy(
            name="/eyrc/vb/conveyor/set_power", service_class=conveyorBeltPowerMsg)
        rospy.loginfo("Conveyor service ready")

    def set_power(self, val):
        """
            Start the conveyor with the set speed.
            val: Speed of the conveyor. (valid range: 11 - 100)
        """
        self._conveyor(val)

    def stop(self):
        """
            Stop the conveyor.
            This intenally sets the speed to 0.
        """
        self._conveyor(0)
