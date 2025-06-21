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
from sensor_msgs.msg import JointState
from rclpy.duration import Duration
import rclpy


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    MIRRORED_BASES = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self, node):
        super().__init__(node)
        topic_prefix = "/joint_trajectory_controller"
        found_topics = self.get_topic_names_and_types(f"{topic_prefix}*/joint_trajectory")

        self.sleep_position = [-1.5708, -0.541052, 0.0, 0.0, 0.0, 0.0]  # Default sleep position for all joints
        self.home_position = [-1.5708, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default home position for all joints
        self.step_size_flexion_joints = 0.001  # joints 2,3,5
        self.step_size_rotation_joints= 0.002  # other joints
        self.flexion_joints_indicies = [1, 2, 4]  # Joints 2, 3, 5
        self.rotation_joints_indicies = [i for i in range(6) if i not in self.flexion_joints_indicies]
        self.mirror_arm = False  # Flag to toggle mirroring of the arm

        self.mirror = self.node.get_parameter("mirror").get_parameter_value().bool_value
        self.joint_trajectory_publishers = {}
        self.topic_to_joint_names = {}
        self.topic_to_commanded_positions = {}
        self.prefix_to_joints = {}
        self.is_joystick_idle = True
        self.num_arms = 0

        # Discover all topics and joint names, extract prefix
        for topic, types in found_topics:#
            self.num_arms += 1
            self.joint_trajectory_publishers[topic] = self.node.create_publisher(
                JointTrajectory, topic, 10
            )
            # Extract prefix from topic name
            # e.g. /joint_trajectory_controller_arm_1/joint_trajectory -> arm_1
            topic_parts = topic[len(topic_prefix) :].split("/")
            prefix = (
                topic_parts[0][1:]
                if topic_parts[0].startswith("_")
                else topic_parts[0] if topic_parts[0] else ""
            )
            joint_names = self.get_param_values(topic.split("/")[1], "joints")
            if joint_names:
                self.topic_to_joint_names[topic] = joint_names
                self.topic_to_commanded_positions[topic] = [0.0] * len(joint_names)
                self.prefix_to_joints[prefix] = joint_names
            else:
                print("Parameter not found or empty for topic", topic)
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

        # Process each topic (arm/controller) independently
        for topic, joint_names in self.topic_to_joint_names.items():
            commanded_positions = self.topic_to_commanded_positions[topic]
            if msg.buttons[self.node.button_mapping["move_home"]]:
                commanded_positions = self.move_to_position(joint_names, self.home_position.copy(), self.mirror_arm)
                any_axis_active = True
                if self.num_arms==2:
                    self.mirror_arm = not self.mirror_arm
            elif msg.buttons[self.node.button_mapping["move_sleep"]]:
                if self.num_arms==2:
                    commanded_positions = self.move_to_position(joint_names, self.sleep_position.copy(), self.mirror_arm)
                    any_axis_active = True
                    self.mirror_arm = not self.mirror_arm
            else:
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
                            commanded_positions[i] = (
                                current_position + self.node.joint_pos_offset_tolerance
                            )
                            self.node.gamepad_feedback.send_feedback(intensity=1.0)
                        elif offset < -self.node.joint_pos_offset_tolerance:
                            commanded_positions[i] = (
                                current_position - self.node.joint_pos_offset_tolerance
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

    def joints_at_home(self, current, indices, target_position, tolerance=0.02):
        return all(abs(current[i] - target_position[i]) < tolerance for i in indices)
    
    # Returns the movement phase based on current joint positions
    # 0 = all joints at home, 1 = flexion joints needs moving, 2 = rotation joints needs moving
    def get_movement_phase(self, current_joints, target_position):

        flexion_done = self.joints_at_home(current_joints, self.flexion_joints_indicies, target_position)
        rotation_done = self.joints_at_home(current_joints, self.rotation_joints_indicies, target_position)

        if flexion_done:
            if rotation_done:
                return 0  # All joints at home
            else:
                return 2  # Only rotation joints needs moving
        else:
            return 1  # flexion joints needs moving
        
    def extract_joint_values(self, all_joint_states, joint_names):
        for joint_dict in all_joint_states:
            if all(name in joint_dict for name in joint_names):
                return [joint_dict[name] for name in joint_names]
        return []  # Not found

    def move_to_position(self, joint_names, target_position, mirror_arm=False):
        if mirror_arm:
            mirrored_indices = [i for i, name in enumerate(joint_names) if any(base in name for base in self.MIRRORED_BASES)]
            for i in mirrored_indices:
                target_position[i] = -target_position[i]
        current_joint_values = self.extract_joint_values(self.get_joint_states(), joint_names)
        self.movement_phase = self.get_movement_phase(current_joint_values, target_position) # Describes which joints need moving, flexion, rotation or all at home

        try:
            if self.movement_phase == 1:
                next_step = self.interpolate_partial(
                    current_joint_values, target_position, self.flexion_joints_indicies, self.step_size_flexion_joints)
                return next_step

            elif self.movement_phase == 2:
                next_step = self.interpolate_partial(
                    current_joint_values, target_position, self.rotation_joints_indicies, self.step_size_rotation_joints)
                return next_step

            elif self.movement_phase == 0:
                self.node.get_logger().info("All joints already at goal position", throttle_duration_sec = 10.0)
                return current_joint_values

        except KeyboardInterrupt:
            print("Keyboard interrupt received, stopping home movement.")

    def interpolate_partial(self, current, target, indices, step_size):
        next_step = current[:]
        for i in indices:
            delta = target[i] - current[i]
            if abs(delta) > step_size:
                next_step[i] = current[i] + step_size * (1 if delta > 0 else -1)
            else:
                next_step[i] = target[i]
        return next_step

    def get_joint_base_name(self, joint_name):
        # Assumes joint names are like 'shoulder_rotation_arm_left'
        return (
            "_".join(joint_name.split("_")[:-2])
            if joint_name.endswith(("_arm_left", "_arm_right"))
            else joint_name.rsplit("_", 1)[0]
        )

    def publish_joint_trajectory(
        self, target_positions, publisher, joint_names=None, speed_percentage=1.0
    ):
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
