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

from dynaarm_extensions.duatic_helpers.duatic_controller_helper import DuaticControllerHelper
from dynaarm_gamepad_interface.controllers.joint_trajectory_controller import JointTrajectoryController
from dynaarm_gamepad_interface.controllers.cartesian_controller import CartesianController
from dynaarm_gamepad_interface.controllers.freedrive_controller import FreedriveController


class ControllerManager:
    """Handle controllers"""

    def __init__(self, node):
        self.node = node

        self.controller_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        
        self.active_high_level_controller_index = -1
        self.active_low_level_controller = None
        self.is_freeze_active = True  # Assume freeze is active until proven otherwise
        self.emergency_button_was_pressed = False        

        self.all_high_level_controllers = {
            0: FreedriveController(self.node),
            1: JointTrajectoryController(self.node),
            2: CartesianController(self.node),
        }

        self.duatic_controller_helper = DuaticControllerHelper(self.node)
        
        self.node.create_timer(0.2, self.check_active_low_level_controllers)

    def get_current_controller(self):
        if self.active_high_level_controller_index < 0:
            return None

        return self.all_high_level_controllers[self.active_high_level_controller_index]

    def wait_for_controller_loaded(self, controller_name, timeout=60.0):
        return self.duatic_controller_helper.wait_for_controller_loaded(controller_name, timeout)

    def wait_for_controller_data(self):
        return self.duatic_controller_helper.wait_for_controller_data()

    def check_active_low_level_controllers(self):
        """Checks which controllers are active and updates the state machine."""

        self.is_freeze_active = self.duatic_controller_helper.is_freeze_active()

        if self.is_freeze_active and not self.emergency_button_was_pressed:            
            self.node.get_logger().warn(
            "       ⚠️   Emergency stop is ACTIVE!   ⚠️           \n"
            "\t\t\t\t\tTo deactivate: Hold Left Stick Button (LSB) or L1 for ~4s.")
            self.emergency_button_was_pressed = True
            self.is_freeze_active = True
        elif not self.is_freeze_active and self.emergency_button_was_pressed:
            self.node.get_logger().warn(
            "    ✅   Emergency stop is DEACTIVATED!   ✅           ")
            self.is_freeze_active = False
            self.emergency_button_was_pressed = False

        active_low_level_controller = self.duatic_controller_helper.get_active_controller()
        
        if not active_low_level_controller:
            self.node.get_logger().warn(
                "No active controller found.", throttle_duration_sec=30.0
            )
            self.active_low_level_controller_index = -1
            self.active_high_level_controller_index = -1
            return

        if active_low_level_controller != self.active_low_level_controller:
            self.active_low_level_controller = active_low_level_controller

            # Only set high-level controller if none was set before
            if self.active_high_level_controller_index == -1:
                for idx, high_level_controller in self.all_high_level_controllers.items():
                    if hasattr(high_level_controller, "get_low_level_controller"):
                        if high_level_controller.get_low_level_controller() == self.active_low_level_controller:
                            self.active_high_level_controller_index = idx
                            high_level_controller.reset()  # Reset the controller to get fresh joint values
                            self.node.get_logger().info(f"Activated high level controller index: {idx} for low level controller: {self.active_low_level_controller}")
                            break

    def switch_to_next_controller(self):
        """Switch to the next high-level controller. Only switch low-level controller if needed."""

        num_bases = len(self.all_high_level_controllers)
        if num_bases <= 0:
            return

        next_high_level_controller_index = (self.active_high_level_controller_index + 1) % num_bases
        next_high_level_controller = self.all_high_level_controllers[next_high_level_controller_index]
        next_low_level_controller = next_high_level_controller.get_low_level_controller()
        
        if next_low_level_controller is None:
            self.node.get_logger().warn(
                "No low-level controller defined for the next high-level controller."
            )
            return

        # Always update the high-level controller index
        self.active_high_level_controller_index = next_high_level_controller_index
        self.node.get_logger().info(
            f"Switching to high-level controller index: {next_high_level_controller_index} ({next_high_level_controller.__class__.__name__})"
        )

        # Only switch low-level controller if it is different
        if next_low_level_controller == self.duatic_controller_helper.active_low_level_controller:
            self.node.get_logger().info(
                f"Already using controller: {next_low_level_controller}"
            )
            return

        # Deactivate all child controllers of the current active base (if any)
        activate_controllers = []
        deactivate_controllers = []

        if self.duatic_controller_helper.active_low_level_controller is not None:
            deactivate_controllers.append(self.duatic_controller_helper.active_low_level_controller)
        activate_controllers.append(next_low_level_controller)

        self.duatic_controller_helper.switch_controller(activate_controllers, deactivate_controllers)
