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
from rclpy.parameter_client import AsyncParameterClient
import re

class BaseController:
    """Base class for all controllers, providing logging and common methods."""

    def __init__(self, node):
        self.node = node
        self.log_printed = False  # Track whether the log was printed
        self.arms_count = 0  # Count of arms, used for logging
    
    def get_param_values(self, controller_ns, param_name):
        """Retrieve parameter values from the node."""

        param_client = AsyncParameterClient(self.node, controller_ns)
        future = param_client.get_parameters([param_name])
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None and future.result().values:
            param_value = future.result().values[0]
            joint_names = list(param_value.string_array_value)
            return joint_names

    def get_topic_names_and_types(self, by_name):
        """ This base class retrieves topic names by a given name.
        Args:
            by_name (str): The name of the topic to retrieve.
        Returns:
            list: A list of topic names and types matching the given name.
        """
        
        pattern = re.compile(by_name.replace("*", ".*"))
        topics_and_types = self.node.get_topic_names_and_types()
        matches = [
            (topic, types)
            for topic, types in topics_and_types
            if pattern.fullmatch(topic)
        ]
        self.arms_count = len(matches)
        return matches

    def get_joint_states(self):
        """Always return a list of joint state dicts, one per arm."""
        joint_states = self.node.joint_states
        if self.arms_count <= 1:
            return [joint_states]  # Always a list, even for single arm

        # Multi-arm: split joint_states into self.arms_count chunks
        joint_names = list(joint_states.keys())
        values = list(joint_states.values())
        chunk_size = len(joint_names) // self.arms_count
        joint_states_per_arm = []
        for i in range(self.arms_count):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            arm_joint_names = joint_names[start:end]
            arm_joint_values = values[start:end]
            arm_joint_dict = dict(zip(arm_joint_names, arm_joint_values))
            joint_states_per_arm.append(arm_joint_dict)
        return joint_states_per_arm

    def process_input(self, joy_msg):
        """Override this in child classes."""
        pass

    def reset(self):
        """Reset controller state when switching back to this controller."""
        self.log_printed = False  # Reset logging state
