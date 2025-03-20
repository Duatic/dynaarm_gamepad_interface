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

        # Publisher for Cartesian pose commands
        self.cartesian_publisher = self.node.create_publisher(
            PoseStamped, "/cartesian_motion_controller/target_frame", 10
        )

        # Utils
        self.tf_helper = TFHelper(self.node)
        self.marker_helper = MarkerHelper(self.node)
        self.sphere_props = MarkerProperties(
            type=Marker.SPHERE,
            scale_x=0.05,
            scale_y=0.05,
            scale_z=0.05,
            color_r=1.0,
            color_g=0.0,
            color_b=0.0,
            color_a=1.0,
        )
        self.arrow_props = MarkerProperties(
            type=Marker.ARROW,
            scale_x=0.1,
            scale_y=0.02,
            scale_z=0.02,
            color_r=0.0,
            color_g=0.0,
            color_b=1.0,
            color_a=1.0,
        )

        # Initialize the target pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.base_frame

    def reset(self):
        """Resets the current_pose to the current one"""

        self.current_pose = self.tf_helper.compute_fk(self.base_frame, self.ee_frame)

    def process_input(self, msg):
        """Processes joystick input and updates Cartesian position."""
        super().process_input(msg)

        dt = self.node.dt  # Get time step from the node

        # Translation Controls
        lx = msg.axes[self.node.axis_mapping["left_joystick"]["x"]]  # Move in X
        ly = msg.axes[self.node.axis_mapping["left_joystick"]["y"]]  # Move in Y
        lz = msg.axes[self.node.axis_mapping["right_joystick"]["y"]]  # Move in Z

        # Rotation Controls
        roll = msg.axes[self.node.axis_mapping["right_joystick"]["x"]]  # Rotate Roll (X-axis)
        pitch = (
            msg.axes[self.node.axis_mapping["triggers"]["left"]]
            - msg.axes[self.node.axis_mapping["triggers"]["right"]]
        )

        # Yaw Rotation Using Buttons
        rotate_left = (
            msg.buttons[self.node.button_mapping["wrist_rotation_left"]]
            if self.node.button_mapping["wrist_rotation_left"] < len(msg.buttons)
            else 0
        )
        rotate_right = (
            msg.buttons[self.node.button_mapping["wrist_rotation_right"]]
            if self.node.button_mapping["wrist_rotation_right"] < len(msg.buttons)
            else 0
        )
        yaw = -1.0 if rotate_left else (1.0 if rotate_right else 0.0)  # Rotate Yaw (Z-axis)

        # Scaling Factors
        linear_speed = 0.2  # Adjust translation speed
        angular_speed = 0.4  # Adjust rotation speed

        # Update Position
        self.current_pose.pose.position.x += lx * linear_speed * dt
        self.current_pose.pose.position.y += ly * linear_speed * dt
        self.current_pose.pose.position.z += lz * linear_speed * dt

        # Update Orientation (Apply Incremental Rotations)
        current_q = self.current_pose.pose.orientation
        current_euler = euler_from_quaternion([current_q.x, current_q.y, current_q.z, current_q.w])

        # Apply joystick inputs as incremental changes
        new_euler = (
            current_euler[0] + (roll * angular_speed * dt),
            current_euler[1] + (pitch * angular_speed * dt),
            current_euler[2] + (yaw * angular_speed * dt),
        )
        new_q = quaternion_from_euler(*new_euler)

        self.current_pose.pose.orientation.x = new_q[0]
        self.current_pose.pose.orientation.y = new_q[1]
        self.current_pose.pose.orientation.z = new_q[2]
        self.current_pose.pose.orientation.w = new_q[3]

        # Publish Cartesian pose and marker
        self.publish_cartesian_command()

        marker_data_list = [
            MarkerData(
                id=1001, pose=self.current_pose, frame=self.base_frame, properties=self.sphere_props
            ),
            MarkerData(
                id=1002, pose=self.current_pose, frame=self.base_frame, properties=self.arrow_props
            ),
        ]

        self.marker_helper.publish_markers(marker_data_list)

    def publish_cartesian_command(self):
        """Publishes a new Cartesian target pose."""
        self.current_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.cartesian_publisher.publish(self.current_pose)
        self.node.get_logger().debug(f"Published Cartesian Pose: {self.current_pose.pose.position}")
