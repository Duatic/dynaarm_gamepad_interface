#!/usr/bin/env python3

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

import os
import sys
import threading
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import ListControllers

from dynaarm_gamepad_interface.controller_manager import ControllerManager
from dynaarm_gamepad_interface.utils.gamepad_feedback import GamepadFeedback


class GamepadInterface(Node):
    """Processes joystick input"""

    def __init__(self, mirror=False):
        super().__init__("gamepad_interface")

        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        self.joint_states = {}
        self.initial_positions_set = False
        self.commanded_positions = []
        self.is_joystick_idle = True
        self.joint_pos_offset_tolerance = 0.1
        self.dt = 0.0005
        self.latest_joy_msg = None
        self.joy_lock = threading.Lock()
        self.last_menu_button_state = 0

        self.declare_parameter("mirror", mirror)

        # Subscribers
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Load gamepad mappings from YAML
        config_path = os.path.join(
            get_package_share_directory("dynaarm_gamepad_interface"),
            "config",
            "gamepad_config.yaml",
        )
        with open(config_path) as file:
            config = yaml.safe_load(file)["gamepad_node"]["ros__parameters"]

        # Load configurations
        self.button_mapping = config["button_mapping"]
        self.axis_mapping = config["axis_mapping"]
        self.get_logger().info(f"Loaded gamepad config: {self.button_mapping}, {self.axis_mapping}")

        self.controller_manager = ControllerManager(self, config["controllers"])
        self.gamepad_feedback = GamepadFeedback(self)

        self.create_timer(self.dt, self.process_joy_input)
        self.get_logger().info("Gamepad Interface Initialized.")

    def joint_state_callback(self, msg: JointState):
        """Update stored joint states and set initial positions."""
        self.joint_states = dict(zip(msg.name, msg.position))

        # Set initial commanded positions only once
        if not self.initial_positions_set and self.joint_states:
            self.commanded_positions = list(self.joint_states.values())
            self.initial_positions_set = True

    def joy_callback(self, msg: Joy):
        """Store latest joystick message."""
        with self.joy_lock:
            self.latest_joy_msg = msg

    def process_joy_input(self):
        """Process latest joystick input."""
        with self.joy_lock:
            msg = self.latest_joy_msg  # Get the latest stored joystick input

        if msg is None or not self.joint_states:
            return  # Skip processing if no joystick input or no joint states

        # Use dynamically loaded menu button index
        switch_controller_index = self.button_mapping["switch_controller"]
        # Ensure switching happens only on button press (down event) and not while held down
        if msg.buttons[switch_controller_index] == 1 and self.last_menu_button_state == 0:
            self.controller_manager.switch_to_next_controller()

        # Wait until button is released (0) before allowing another switch
        # And don't execute anything else when the button is pressed
        self.last_menu_button_state = msg.buttons[switch_controller_index]
        if self.last_menu_button_state:
            # TODO Hold current position?
            return

        # Now get the current active controller from the controller manager:
        current_controller = self.controller_manager.get_current_controller()
        if current_controller is not None:
            if self.controller_manager.is_freeze_active:
                # If freeze is active, we don't process any input
                current_controller.reset()
            else:
                current_controller.process_input(msg)

    def wait_for_controller_manager(self):
        self.get_logger().info("Waiting for controller_manager to be online...")
        if not self.controller_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("controller_manager service not available. Exiting.")
            rclpy.shutdown()
            sys.exit(1)
        self.get_logger().info("controller_manager is online.")

    def check_joint_trajectory_controllers(self):
        req = ListControllers.Request()
        future = self.controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            controllers = future.result().controller
            jtcs = [c for c in controllers if c.name.startswith("freeze_controller")]
            count = len(jtcs)
            self.get_logger().info(f"Found {count} robots(s).")
            if count > 2:
                self.get_logger().error(
                    "More than 2 freeze_controllers found. Only up to two are supported."
                )
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Failed to get controllers from controller_manager.")
            rclpy.shutdown()
            sys.exit(1)

    def run(self):
        self.wait_for_controller_manager()
        self.check_joint_trajectory_controllers()


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument("--mirror", action="store_true", help="Mirror arm movements")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = GamepadInterface(mirror=parsed_args.mirror)
    node.run()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
