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
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import ListControllers, SwitchController
from transitions import Machine

# Import controller classes dynamically
from dynaarm_gamepad_interface.controllers.joint_trajectory_controller import (
    JointTrajectoryController,
)
from dynaarm_gamepad_interface.controllers.position_controller import PositionController
from dynaarm_gamepad_interface.controllers.cartesian_controller import CartesianController
from dynaarm_gamepad_interface.controllers.freedrive_controller import FreedriveController


class GamepadBase(Node):
    """Handles state switching, controller management, and gamepad input using a state machine."""

    # Controller name → Class mapping
    CONTROLLER_CLASS_MAP = {
        "freedrive_controller": FreedriveController,
        "joint_trajectory_controller": JointTrajectoryController,
        "cartesian_motion_controller": CartesianController,
        "position_controller": PositionController,
    }

    def __init__(self):
        super().__init__("gamepad_base")

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
        self.controllers_config = config["controllers"]

        self.get_logger().info(f"Loaded gamepad config: {self.button_mapping}, {self.axis_mapping}")

        # Extract controller states & whitelist from config
        self.states = list(self.controllers_config.keys())  # Use controller names as states
        self.controller_whitelist = [
            name for name, props in self.controllers_config.items() if props["whitelisted"]
        ]

        # Track active controller index
        self.current_controller_index = 0

        # Initialize state machine with dynamically loaded states
        self.machine = Machine(model=self, states=self.states, initial="no_control")

        # Instantiate controllers dynamically
        self.controllers = {}
        for controller_name in self.controller_whitelist:
            if controller_name in self.CONTROLLER_CLASS_MAP:
                self.controllers[controller_name] = self.CONTROLLER_CLASS_MAP[controller_name](self)
            else:
                self.get_logger().warn(
                    f"Controller '{controller_name}' is in whitelist but has no mapped class."
                )

        self.get_logger().info(f"Loaded controllers: {list(self.controllers.keys())}")

        # Dictionary to store latest joint states
        self.joint_states = {}

        # Subscribers
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Clients
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.timer = self.create_timer(1.0, self.check_active_controllers)

        self.last_menu_button_state = 0  # Prevents repeated activation on button hold

        self.get_logger().info(f"GamepadBase Node started in mode: {self.state}")

    def joint_state_callback(self, msg: JointState):
        """Update stored joint states when a new message is received."""
        self.joint_states = dict(zip(msg.name, msg.position))  # Store as dictionary
        self.get_logger().debug("Received joint state update.", throttle_duration_sec=5.0)

    def joy_callback(self, msg: Joy):
        """Process joystick input and handle mode switching."""
        pressed_buttons = [i for i, val in enumerate(msg.buttons) if val == 1]

        if pressed_buttons:
            self.get_logger().debug(f"Pressed buttons: {pressed_buttons}")

        # Use dynamically loaded menu button index
        switch_controller_index = self.button_mapping["switch_controller"]
        # Ensure switching happens only on button press (down event) and not while held down
        if msg.buttons[switch_controller_index] == 1 and self.last_menu_button_state == 0:
            self.switch_to_next_controller()

        # Wait until button is released (0) before allowing another switch
        # And don't execute anything else when the button is pressed
        self.last_menu_button_state = msg.buttons[switch_controller_index]
        if self.last_menu_button_state:
            # TODO Hold current position?
            return

        self.process_input(msg)

    def process_input(self, joy_msg: Joy):
        """Dispatch joystick input to the active controller."""
        active_controller = self.state  # Current state from the state machine

        if active_controller == "no_control":
            self.get_logger().warn("No controller active.", throttle_duration_sec=20.0)
        elif active_controller in self.controllers:
            self.controllers[active_controller].process_input(joy_msg)
        else:
            self.get_logger().warn(
                f"Unknown control mode: {active_controller}", throttle_duration_sec=20.0
            )

    def switch_to_next_controller(self):
        """Switch to the next available controller in the whitelist."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "SwitchController service not available.", throttle_duration_sec=10.0
            )
            return

        new_controller_index = (self.current_controller_index + 1) % len(self.controller_whitelist)
        new_controller = self.controller_whitelist[new_controller_index]

        req = SwitchController.Request()
        req.activate_controllers = [new_controller]
        req.deactivate_controllers = list()
        if self.state != "no_control":
            req.deactivate_controllers.append(self.state)
        req.strictness = 1  # BEST EFFORT mode

        future = self.switch_controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.ok:
                    self.check_active_controllers()  # Update state machine
                else:
                    self.get_logger().error(
                        f"Failed to switch to {new_controller}", throttle_duration_sec=10.0
                    )
            except Exception as e:
                self.get_logger().error(
                    f"Error switching controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)

    def check_active_controllers(self):
        """Checks which controllers are active and updates the state machine."""
        if not self.controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "Controller manager service not available.", throttle_duration_sec=10.0
            )
            return

        req = ListControllers.Request()
        future = self.controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()

                # Filter only whitelisted active controllers
                active_controllers = {
                    controller.name
                    for controller in response.controller
                    if controller.state == "active" and controller.name in self.controller_whitelist
                }

                # Check if `freeze_controller` (E-Stop) is active, even though it's NOT in the whitelist
                is_freeze_active = any(
                    controller.name == "freeze_controller" and controller.state == "active"
                    for controller in response.controller
                )

                if is_freeze_active:
                    self.get_logger().warn(
                        "       ⚠️   Emergency stop is ACTIVE!   ⚠️           \n"
                        "\t\t\t\t\tTo deactivate: Hold Left Stick Button (LSB) or L1 for ~4s.",
                        throttle_duration_sec=60.0,
                    )

                # Update state machine based on the active controller
                if active_controllers:
                    new_state = next(iter(active_controllers))  # Get first active controller
                    if new_state in self.machine.states and self.state not in active_controllers:
                        self.get_logger().info(f"Switched controller to: {new_state}")
                        self.machine.set_state(new_state)
                        self.controllers[new_state].reset()

                        # Update the current_controller_index to match the new state
                        if new_state in self.controller_whitelist:
                            self.current_controller_index = self.controller_whitelist.index(
                                new_state
                            )

                else:
                    if self.state != "no_control":
                        self.get_logger().info(
                            "No active controller found. Switching to NO_CONTROL."
                        )
                        self.machine.set_state("no_control")
                        self.current_controller_index = -1

            except Exception as e:
                self.get_logger().error(
                    f"Failed to list controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
