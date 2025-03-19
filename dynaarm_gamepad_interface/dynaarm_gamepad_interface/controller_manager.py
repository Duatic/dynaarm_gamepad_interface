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

from rclpy.node import Node

from controller_manager_msgs.srv import ListControllers, SwitchController

# Import controller classes dynamically
from dynaarm_gamepad_interface.controllers.joint_trajectory_controller import JointTrajectoryController
from dynaarm_gamepad_interface.controllers.position_controller import PositionController
from dynaarm_gamepad_interface.controllers.cartesian_controller import CartesianController
from dynaarm_gamepad_interface.controllers.freedrive_controller import FreedriveController


class ControllerManager():
    """Handle controllers"""

    # Controller name → Class mapping
    CONTROLLER_CLASS_MAP = {
        "freedrive_controller": FreedriveController,
        "joint_trajectory_controller": JointTrajectoryController,
        "cartesian_motion_controller": CartesianController,
        "position_controller": PositionController,
    }

    def __init__(self, node, controllers_config):
        self.node = node

        self.controllers = {}
        self.current_controller_index = -1
        self.current_active_controller = None

        self.controller_client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_controller_client = self.node.create_client(SwitchController, "/controller_manager/switch_controller")

        # Extract controller states & whitelist from config
        self.states = list(controllers_config.keys())  # Use controller names as states
        self.controller_whitelist = [
            name for name, props in controllers_config.items() if props["whitelisted"]
        ]

        # Instantiate controllers dynamically
        for controller_name in self.controller_whitelist:
            if controller_name in self.CONTROLLER_CLASS_MAP:
                self.controllers[controller_name] = self.CONTROLLER_CLASS_MAP[controller_name](self.node)
            else:
                self.node.get_logger().warn(
                    f"Controller '{controller_name}' is in whitelist but has no mapped class."
                )

        self.node.get_logger().info(f"Loaded controllers: {list(self.controllers.keys())}")
        self.node.create_timer(0.5, self.check_active_controllers)

    def get_current_controller(self):
        if self.controller_whitelist:
            controller_name = self.controller_whitelist[self.current_controller_index]
            return self.controllers.get(controller_name, None)
        return None

    def check_active_controllers(self):
        """Checks which controllers are active and updates the state machine."""
        if not self.controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
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
                    self.node.get_logger().warn(
                        "       ⚠️   Emergency stop is ACTIVE!   ⚠️           \n"
                        "\t\t\t\t\tTo deactivate: Hold Left Stick Button (LSB) or L1 for ~4s.",
                        throttle_duration_sec=60.0,
                    )
                
                if active_controllers:
                    
                    # Get first active controller                                      
                    current_active_controller = next(iter(active_controllers))
                    if current_active_controller in self.controller_whitelist:
                        active_controller_index = self.controller_whitelist.index(current_active_controller)
                        if self.current_controller_index != active_controller_index:
                            self.controllers[current_active_controller].reset()
                            self.current_controller_index = active_controller_index
                            self.current_active_controller = current_active_controller                            
                            self.node.get_logger().info(f"New active controller: {current_active_controller}")
                else:
                    self.node.get_logger().warn("No active controller found.", throttle_duration_sec=30.0)
                    self.current_controller_index = -1                        

            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to list controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)

    def switch_to_next_controller(self):
        """Switch to the next available controller in the whitelist."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "SwitchController service not available.", throttle_duration_sec=10.0
            )
            return

        new_controller_index = (self.current_controller_index + 1) % len(self.controller_whitelist)
        new_controller = self.controller_whitelist[new_controller_index]

        req = SwitchController.Request()
        req.activate_controllers = [new_controller]        
        req.deactivate_controllers = list()       
        if self.current_controller_index >= 0:
            req.deactivate_controllers.append(self.current_active_controller)
        req.strictness = 1

        future = self.switch_controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.ok:
                    self.check_active_controllers()  # Update state machine
                else:
                    self.node.get_logger().error(
                        f"Failed to switch to {new_controller}", throttle_duration_sec=10.0
                    )
            except Exception as e:
                self.node.get_logger().error(
                    f"Error switching controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)