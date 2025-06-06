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

from dynaarm_gamepad_interface.controllers.base_controller import BaseController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    MIRRORED_JOINTS = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self, node):
        super().__init__(node)
        topic_prefix = "/joint_trajectory_controller"
        found_topics = self.get_topic_names_and_types(f"{topic_prefix}*/joint_trajectory")

        self.mirror = self.node.get_parameter("mirror").get_parameter_value().bool_value        
        self.joint_trajectory_publishers = {}
        self.topic_to_joint_names = {}
        self.topic_to_commanded_positions = {}

        for topic, types in found_topics:
            self.joint_trajectory_publishers[topic] = self.node.create_publisher(
                JointTrajectory, topic, 10
            )

            joint_names = self.get_param_values(topic.split("/")[1], "joints")
            if joint_names:
                self.topic_to_joint_names[topic] = joint_names
                self.topic_to_commanded_positions[topic] = [0.0] * len(joint_names)
            else:
                print("Parameter not found or empty")
        
        self.is_joystick_idle = True  # Track joystick idle state
        self.commanded_positions = []  # Stores the current commanded positions

        self.mirrored_joints = []
        all_joint_names = []
        for joint_names in self.topic_to_joint_names.values():
            all_joint_names.extend(joint_names)
        # For each base, find all matching joints
        for base in self.MIRRORED_JOINTS:
            matches = [name for name in all_joint_names if name.endswith(base)]
            if len(matches) == 2:
                # Add only one (e.g. the one with 'right' if present, else the first)
                right = [m for m in matches if "right" in m]
                if right:
                    self.mirrored_joints.append(right[0])
                else:
                    self.mirrored_joints.append(matches[0])        

    def reset(self):
        """Reset commanded positions to current joint states for all topics."""
        joint_states_list = self.get_joint_states()  # Returns a list of dicts

        for topic, joint_names in self.topic_to_joint_names.items():
            # Determine which dict to use based on topic
            if "arm_left" in topic:
                arm_joint_states = next((d for d in joint_states_list if any(k.startswith("arm_left") for k in d)), {})
            elif "arm_right" in topic:
                arm_joint_states = next((d for d in joint_states_list if any(k.startswith("arm_right") for k in d)), {})
            else:
                arm_joint_states = {}

            self.topic_to_commanded_positions[topic] = [
                arm_joint_states.get(joint, 0.0) for joint in joint_names
            ]                       

    def process_input(self, msg):
        """Processes joystick input, integrates over dt, and clamps the commanded positions."""
        super().process_input(msg)  # For any base logging logic

        any_axis_active = False
        deadzone = 0.1

        # Process each topic (arm/controller) independently
        for topic, joint_names in self.topic_to_joint_names.items():
            commanded_positions = self.topic_to_commanded_positions[topic]
            for i, joint_name in enumerate(joint_names):
                axis_val = 0.0

                # Map axes/buttons as needed for each joint index
                if i == 0 and len(msg.axes) > self.node.axis_mapping["left_joystick"]["x"]:
                    axis_val = msg.axes[self.node.axis_mapping["left_joystick"]["x"]]
                elif i == 1 and len(msg.axes) > self.node.axis_mapping["left_joystick"]["y"]:
                    axis_val = msg.axes[self.node.axis_mapping["left_joystick"]["y"]]
                elif i == 2 and len(msg.axes) > self.node.axis_mapping["right_joystick"]["y"]:
                    axis_val = msg.axes[self.node.axis_mapping["right_joystick"]["y"]]
                elif i == 3 and len(msg.axes) > self.node.axis_mapping["right_joystick"]["x"]:
                    axis_val = msg.axes[self.node.axis_mapping["right_joystick"]["x"]]
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

                if abs(axis_val) > deadzone:
                    current_position = self.node.joint_states.get(joint_name, 0.0)
                    commanded_positions[i] += axis_val * self.node.dt
                    offset = commanded_positions[i] - current_position
                    if offset > self.node.joint_pos_offset_tolerance:
                        commanded_positions[i] = current_position + self.node.joint_pos_offset_tolerance
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    elif offset < -self.node.joint_pos_offset_tolerance:
                        commanded_positions[i] = current_position - self.node.joint_pos_offset_tolerance
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
                    joint_names=self.topic_to_joint_names[topic]
                )
        elif not any_axis_active and not self.is_joystick_idle:
            # Hold position for all topics
            for topic, publisher in self.joint_trajectory_publishers.items():
                self.publish_joint_trajectory(
                    self.topic_to_commanded_positions[topic],
                    publisher=publisher,
                    joint_names=self.topic_to_joint_names[topic]
                )
            self.is_joystick_idle = True        

    def get_joint_base_name(self, joint_name):
        # Assumes joint names are like 'shoulder_rotation_arm_left'
        return "_".join(joint_name.split("_")[:-2]) if joint_name.endswith(("_arm_left", "_arm_right")) else joint_name.rsplit("_", 1)[0]

    def publish_joint_trajectory(self, target_positions, publisher, joint_names=None, speed_percentage=1.0):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""
        if joint_names is None:
            joint_names = list(self.node.joint_states.keys())

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
