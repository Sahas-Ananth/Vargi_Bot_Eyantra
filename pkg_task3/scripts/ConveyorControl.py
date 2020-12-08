#! /usr/bin/env python

import rospy
from pkg_vb_sim.srv import *

class Conveyor:
    def __init__(self):
        self._conveyor = rospy.ServiceProxy(
            name="/eyrc/vb/conveyor/set_power", service_class=conveyorBeltPowerMsg)

        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        rospy.loginfo("Conveyor service ready")
    
    def set_power(self, val):
        self._conveyor(val)

    def stop(self):
        self._conveyor(0)
