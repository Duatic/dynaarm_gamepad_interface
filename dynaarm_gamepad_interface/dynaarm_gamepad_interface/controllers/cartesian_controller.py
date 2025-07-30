# Copyright 2025 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from dynaarm_gamepad_interface.controllers.base_controller import BaseController
from dynaarm_gamepad_interface.utils.tf_helper import TFHelper
from dynaarm_gamepad_interface.utils.marker_visualizer import (
    MarkerHelper,
    MarkerProperties,
    MarkerData,
)


class CartesianController(BaseController):
    """Handles Cartesian control mode and publishes a visualization marker."""

    def __init__(self, node):
        super().__init__(node)

        self.base_frame = "base"
        self.ee_frame = "flange"

        self.controller_base_name = "joint_trajectory_controller"

        # Publisher for Cartesian pose commands
        self.cartesian_publisher = self.node.create_publisher(
            PoseStamped, "/cartesian_motion_controller/target_frame", 10
        )

    def reset(self):
        """Resets the current_pose to the current one"""

        pass

    def process_input(self, msg):
        """Processes joystick input and updates Cartesian position."""
        super().process_input(msg)

        pass

    def publish_cartesian_command(self):
        """Publishes a new Cartesian target pose."""
        pass
