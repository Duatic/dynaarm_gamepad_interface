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

import rclpy

from dynaarm_extensions.duatic_helpers.duatic_jtc_helper import DuaticJTCHelper
from dynaarm_gamepad_interface.controllers.base_controller import BaseController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    MIRRORED_BASES = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self, node):
        super().__init__(node)

        self.controller_base_name = "joint_trajectory_controller"

        self.arms_count = self.robot_helper.get_robot_count()
        while self.arms_count <= 0:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            self.arms_count = self.robot_helper.get_robot_count()

        duatic_jtc_helper = DuaticJTCHelper(self.node, self.arms_count)
        found_topics = duatic_jtc_helper.get_joint_trajectory_topics()
        response = duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_positions = response[1]

        # Create publishers for each joint trajectory topic
        self.joint_trajectory_publishers = {}

        for topic in self.topic_to_joint_names.keys():
            self.joint_trajectory_publishers[topic] = self.node.create_publisher(
                JointTrajectory, topic, 10
            )
            self.node.get_logger().info(f"Created publisher for topic: {topic}")

        self.mirror = self.node.get_parameter("mirror").get_parameter_value().bool_value

        self.prefix_to_joints = {}
        self.is_joystick_idle = True

        # Dominant axis tracking for smoother joystick control
        self.dominant_axis_threshold = 0.4  # Higher threshold for non-dominant axis
        self.active_axes = {
            "left_joystick": {"x": False, "y": False},
            "right_joystick": {"x": False, "y": False},
        }

        # Build mirrored joints: for each base, find all prefixes that have that joint
        self.mirrored_joints = []
        for base in self.MIRRORED_BASES:
            # Find all joints with this base, grouped by prefix
            found = []
            for prefix, joints in self.prefix_to_joints.items():
                for joint in joints:
                    # Accept both arm_1/shoulder_rotation and arm_1_shoulder_rotation
                    if joint.endswith("/" + base) or joint.endswith("_" + base) or joint == base:
                        found.append(joint)
            if len(found) == 2:
                # Pick one to mirror (e.g. the one with the "higher" prefix)
                found_sorted = sorted(found)
                self.mirrored_joints.append(found_sorted[1])
        if self.mirrored_joints:
            self.node.get_logger().info(f"Mirrored joints: {self.mirrored_joints}")

    def reset(self):
        """Reset commanded positions to current joint states for all topics."""
        joint_states_list = self.get_joint_states()  # Returns a list of dicts

        for topic, joint_names in self.topic_to_joint_names.items():
            # Find the dict with the most matching joint names
            best_dict = {}
            max_found = 0
            for d in joint_states_list:
                found = sum(1 for joint in joint_names if joint in d)
                if found > max_found:
                    max_found = found
                    best_dict = d
            # Use best_dict for this topic
            self.topic_to_commanded_positions[topic] = [
                best_dict.get(joint, 0.0) for joint in joint_names
            ]

    def process_input(self, msg):
        """Processes joystick input, integrates over dt, and clamps the commanded positions."""
        super().process_input(msg)  # For any base logging logic
        
        any_axis_active = False
        deadzone = 0.1

        # Get raw joystick values for dominant axis calculation
        left_x = (
            msg.axes[self.node.axis_mapping["left_joystick"]["x"]]
            if len(msg.axes) > self.node.axis_mapping["left_joystick"]["x"]
            else 0.0
        )
        left_y = (
            msg.axes[self.node.axis_mapping["left_joystick"]["y"]]
            if len(msg.axes) > self.node.axis_mapping["left_joystick"]["y"]
            else 0.0
        )
        right_x = (
            msg.axes[self.node.axis_mapping["right_joystick"]["x"]]
            if len(msg.axes) > self.node.axis_mapping["right_joystick"]["x"]
            else 0.0
        )
        right_y = (
            msg.axes[self.node.axis_mapping["right_joystick"]["y"]]
            if len(msg.axes) > self.node.axis_mapping["right_joystick"]["y"]
            else 0.0
        )

        # Determine which axes are currently dominant
        self._update_dominant_axes(left_x, left_y, right_x, right_y, deadzone)

        # Process each topic (arm/controller) independently
        for topic, joint_names in self.topic_to_joint_names.items():
            commanded_positions = self.topic_to_commanded_positions[topic]
            for i, joint_name in enumerate(joint_names):
                axis_val = 0.0
                effective_deadzone = deadzone

                # Map axes/buttons as needed for each joint index and apply dominant axis logic
                if i == 0 and len(msg.axes) > self.node.axis_mapping["left_joystick"]["x"]:
                    axis_val = left_x
                    # If left Y is dominant, increase deadzone for left X
                    if (
                        self.active_axes["left_joystick"]["y"]
                        and not self.active_axes["left_joystick"]["x"]
                    ):
                        effective_deadzone = self.dominant_axis_threshold
                elif i == 1 and len(msg.axes) > self.node.axis_mapping["left_joystick"]["y"]:
                    axis_val = left_y
                    # If left X is dominant, increase deadzone for left Y
                    if (
                        self.active_axes["left_joystick"]["x"]
                        and not self.active_axes["left_joystick"]["y"]
                    ):
                        effective_deadzone = self.dominant_axis_threshold
                elif i == 2 and len(msg.axes) > self.node.axis_mapping["right_joystick"]["y"]:
                    axis_val = right_y
                    # If right X is dominant, increase deadzone for right Y
                    if (
                        self.active_axes["right_joystick"]["x"]
                        and not self.active_axes["right_joystick"]["y"]
                    ):
                        effective_deadzone = self.dominant_axis_threshold
                elif i == 3 and len(msg.axes) > self.node.axis_mapping["right_joystick"]["x"]:
                    axis_val = right_x
                    # If right Y is dominant, increase deadzone for right X
                    if (
                        self.active_axes["right_joystick"]["y"]
                        and not self.active_axes["right_joystick"]["x"]
                    ):
                        effective_deadzone = self.dominant_axis_threshold
                elif i == 4:
                    left_trigger = msg.axes[self.node.axis_mapping["triggers"]["left"]]
                    right_trigger = msg.axes[self.node.axis_mapping["triggers"]["right"]]
                    axis_val = right_trigger - left_trigger
                elif i == 5:
                    move_left = msg.buttons[self.node.button_mapping["wrist_rotation_left"]] == 1
                    move_right = msg.buttons[self.node.button_mapping["wrist_rotation_right"]] == 1
                    if move_left and not move_right:
                        axis_val = -1.0
                    elif move_right and not move_left:
                        axis_val = 1.0

                if self.mirror and joint_name in self.mirrored_joints:
                    axis_val = -axis_val

                if abs(axis_val) > effective_deadzone:                    
                    current_position = self.get_joint_value_from_states(joint_name)
                    commanded_positions[i] += axis_val * self.node.dt
                    offset = commanded_positions[i] - current_position
                    if offset > self.joint_pos_offset_tolerance:
                        commanded_positions[i] = (
                            current_position + self.joint_pos_offset_tolerance
                        )
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    elif offset < -self.joint_pos_offset_tolerance:
                        commanded_positions[i] = (
                            current_position - self.joint_pos_offset_tolerance
                        )
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    any_axis_active = True

            # Update the commanded positions for this topic
            self.topic_to_commanded_positions[topic] = commanded_positions

        # Publish for all topics that have movement
        if any_axis_active:
            self.is_joystick_idle = False
            for topic, publisher in self.joint_trajectory_publishers.items():
                self.publish_joint_trajectory(
                    self.topic_to_commanded_positions[topic],
                    publisher=publisher,
                    joint_names=self.topic_to_joint_names[topic],
                )
        elif not any_axis_active and not self.is_joystick_idle:
            # Hold position for all topics
            for topic, publisher in self.joint_trajectory_publishers.items():
                self.publish_joint_trajectory(
                    self.topic_to_commanded_positions[topic],
                    publisher=publisher,
                    joint_names=self.topic_to_joint_names[topic],
                )
            self.is_joystick_idle = True

    def publish_joint_trajectory(
        self, target_positions, publisher, joint_names, speed_percentage=1.0
    ):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""
        
        if not joint_names:
            self.node.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.node.get_logger().error("No trajectory points available to publish.")
            return

        # Clamp speed percentage between 1 and 100
        speed_percentage = max(1.0, min(100.0, speed_percentage))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        time_in_sec = self.node.dt
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def _update_dominant_axes(self, left_x, left_y, right_x, right_y, deadzone):
        """Update which axes are currently active to determine dominant axis behavior."""
        # Update active axes based on current input
        self.active_axes["left_joystick"]["x"] = abs(left_x) > deadzone
        self.active_axes["left_joystick"]["y"] = abs(left_y) > deadzone
        self.active_axes["right_joystick"]["x"] = abs(right_x) > deadzone
        self.active_axes["right_joystick"]["y"] = abs(right_y) > deadzone
