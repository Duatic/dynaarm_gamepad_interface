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


from controller_manager_msgs.srv import ListControllers, SwitchController


from dynaarm_gamepad_interface.controllers.joint_trajectory_controller import (
    JointTrajectoryController,
)
from dynaarm_gamepad_interface.controllers.position_controller import PositionController
from dynaarm_gamepad_interface.controllers.cartesian_controller import CartesianController
from dynaarm_gamepad_interface.controllers.freedrive_controller import FreedriveController


class ControllerManager:
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
        self.is_freeze_active = True  # Assume freeze is active until proven otherwise

        self.controller_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.controller_whitelist = [
            name for name, props in controllers_config.items() if props["whitelisted"]
        ]

        # Instantiate controllers dynamically
        for controller_name in self.controller_whitelist:
            if controller_name in self.CONTROLLER_CLASS_MAP:
                self.controllers[controller_name] = self.CONTROLLER_CLASS_MAP[controller_name](
                    self.node
                )
            else:
                self.node.get_logger().warn(
                    f"Controller '{controller_name}' is in whitelist but has no mapped class."
                )

        self.node.get_logger().info(f"Loaded controllers: {list(self.controllers.keys())}")
        self.node.create_timer(0.5, self.check_active_controllers)

        # Dict: base_name -> list of found controllers with that base
        self.found_controllers_by_base = {base: [] for base in self.controller_whitelist}

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

                # Reset found controllers by base
                for base in self.found_controllers_by_base:
                    self.found_controllers_by_base[base] = []

                # Populate found controllers by base name
                for controller in response.controller:
                    for base in self.controller_whitelist:
                        if controller.name.startswith(base):
                            self.found_controllers_by_base[base].append(
                                {controller.name: controller.state == "active"}
                            )

                # Filter only whitelisted active controllers
                active_controller = {
                    base
                    for base, controllers in self.found_controllers_by_base.items()
                    for controller_dict in controllers
                    for name, is_active in controller_dict.items()
                    if is_active
                }

                # Check if `freeze_controller` (E-Stop) is active, even though it's NOT in the whitelist
                self.is_freeze_active = any(
                    controller.name.startswith("freeze_controller") and controller.state == "active"
                    for controller in response.controller
                )

                if self.is_freeze_active:
                    self.node.get_logger().warn(
                        "       ⚠️   Emergency stop is ACTIVE!   ⚠️           \n"
                        "\t\t\t\t\tTo deactivate: Hold Left Stick Button (LSB) or L1 for ~4s.",
                        throttle_duration_sec=60.0,
                    )

                if active_controller:

                    # Get first active controller
                    current_active_controller = next(iter(active_controller))

                    # If the current active controller is in the whitelist, update the state
                    if (
                        current_active_controller in self.found_controllers_by_base
                        and self.found_controllers_by_base[current_active_controller]
                    ):
                        active_controller_index = self.controller_whitelist.index(current_active_controller)
                        
                        if self.current_controller_index != active_controller_index:
                    
                            self.controllers[current_active_controller].reset()                            
                            self.current_controller_index = active_controller_index
                            self.current_active_controller = current_active_controller

                            self.node.get_logger().info(
                                f"New active controller: {current_active_controller}"
                            )

                else:
                    self.node.get_logger().warn(
                        "No active controller found.", throttle_duration_sec=30.0
                    )
                    self.current_controller_index = -1

            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to list controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)

    def switch_to_next_controller(self):
        """Switch to the next available controller in the whitelist, using only found controllers."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "SwitchController service not available.", throttle_duration_sec=10.0
            )
            return

        # Try to find the next base controller in the whitelist that has real controllers
        num_bases = len(self.controller_whitelist)
        for i in range(1, num_bases + 1):
            new_base_index = (self.current_controller_index + i) % num_bases
            new_base = self.controller_whitelist[new_base_index]
            real_controllers = [
            name for controllers in self.found_controllers_by_base.get(new_base, [])
            for name, _ in controllers.items()
            ]
            
            if real_controllers:
                break

            self.node.get_logger().debug(f"Real controllers for base '{new_base}': {real_controllers}")
        else:
            self.node.get_logger().warn(
            "No real controllers found for any base in the whitelist.", throttle_duration_sec=10.0
            )
            return

        req = SwitchController.Request()
        req.activate_controllers = real_controllers

        # Deactivate all child controllers of the current active base (if any)
        req.deactivate_controllers = []
        if self.current_controller_index >= 0:
            current_base = self.controller_whitelist[self.current_controller_index]
            current_real_controllers = [
                name for controllers in self.found_controllers_by_base.get(current_base, [])
                for name, _ in controllers.items()
            ]
            req.deactivate_controllers.extend(current_real_controllers)

        req.strictness = 1

        future = self.switch_controller_client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.ok:
                    self.check_active_controllers()
                else:
                    self.node.get_logger().error(
                        f"Failed to switch to {new_base}", throttle_duration_sec=10.0
                    )
            except Exception as e:
                self.node.get_logger().error(
                    f"Error switching controllers: {e}", throttle_duration_sec=10.0
                )

        future.add_done_callback(callback)
