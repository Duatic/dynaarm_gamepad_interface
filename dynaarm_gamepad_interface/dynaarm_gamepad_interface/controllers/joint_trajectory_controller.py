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


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    def __init__(self, node):
        super().__init__(node)

        # Publisher for sending joint trajectory commands
        self.joint_trajectory_publisher = self.node.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.is_joystick_idle = True  # Track joystick idle state        
        self.commanded_positions = []  # Stores the current commanded positions

    def reset(self):
        """Reset commanded positions to current joint states on activation."""
        joint_states = self.get_joint_states()
        if joint_states:
            self.commanded_positions = list(joint_states.values())            

    def process_input(self, msg):
        """Processes joystick input, integrates over dt, and clamps the commanded positions."""
        super().process_input(msg)  # For any base logging logic

        joint_names = list(self.node.joint_states.keys())

        any_axis_active = False
        deadzone = 0.1

        for i, joint_name in enumerate(joint_names):
            axis_val = 0.0   

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
                move_left = (self.node.button_mapping["wrist_rotation_left"] < len(msg.buttons)
                             and msg.buttons[self.node.button_mapping["wrist_rotation_left"]] == 1)
                move_right = (self.node.button_mapping["wrist_rotation_right"] < len(msg.buttons)
                              and msg.buttons[self.node.button_mapping["wrist_rotation_right"]] == 1)
                if move_left and not move_right:
                    axis_val = -1.0
                elif move_right and not move_left:
                    axis_val = 1.0

            # Only process if axis input is beyond deadzone
            if abs(axis_val) > deadzone:
                current_position = self.node.joint_states[joint_name]

                self.commanded_positions[i] += axis_val * self.node.dt 

                # Clamp the offset between the commanded and current positions:
                offset = self.commanded_positions[i] - current_position
                if offset > self.node.joint_pos_offset_tolerance:                    
                    self.commanded_positions[i] = current_position + self.node.joint_pos_offset_tolerance
                    self.node.gamepad_feedback.send_feedback(intensity=1.0)
                elif offset < -self.node.joint_pos_offset_tolerance:                    
                    self.commanded_positions[i] = current_position - self.node.joint_pos_offset_tolerance
                    self.node.gamepad_feedback.send_feedback(intensity=1.0)


                any_axis_active = True
        
        # Publish position command if movement detected
        if any_axis_active:
            self.is_joystick_idle = False
            self.publish_joint_trajectory(self.commanded_positions)

        # If joystick was just released, hold position
        elif not any_axis_active and not self.is_joystick_idle:
            self.publish_joint_trajectory(self.commanded_positions)
            self.is_joystick_idle = True  # Mark as idle

    def publish_joint_trajectory(self, target_positions, speed_percentage = 1.0):
        """Publishes a joint trajectory message with multiple points for smoother movement."""
        joint_names = list(self.get_joint_states().keys())

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
        point.velocities = [0.0] * len(joint_names)  # Set velocity to zero
        point.accelerations = [0.0] * len(joint_names)  # Set acceleration to zero
        time_in_sec = self.node.dt
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        self.joint_trajectory_publisher.publish(trajectory_msg)        
