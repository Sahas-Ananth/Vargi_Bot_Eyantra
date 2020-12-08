#! /usr/bin/env python

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
        self._conveyor = rospy.ServiceProxy(
            name="/eyrc/vb/conveyor/set_power", service_class=conveyorBeltPowerMsg)

        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
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
