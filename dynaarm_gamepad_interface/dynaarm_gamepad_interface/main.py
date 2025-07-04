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
import time
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import ListControllers

from dynaarm_gamepad_interface.controller_manager import ControllerManager, SwitchController
from dynaarm_gamepad_interface.utils.gamepad_feedback import GamepadFeedback


class GamepadInterface(Node):
    """Processes joystick input"""

    def __init__(self, mirror=False):
        super().__init__("gamepad_interface")

        self.hardware_service = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.get_logger().info("Waiting for hardware service...")
        while not self.hardware_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("Still waiting for hardware...")
        self.get_logger().info("Hardware is ready! Starting gamepad interface.")

        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        self.wait_for_any_joint_trajectory_controller_active()

        self.is_simulation = self._check_simulation_mode()
        self.joint_states = {}
        self.initial_positions_set = False
        self.commanded_positions = []
        self.is_joystick_idle = True
        self.joint_pos_offset_tolerance = 0.1

        if self.is_simulation:
            self.dt = 0.05  # 50ms for simulation (20Hz)
            self.get_logger().info("Using simulation timing: dt=0.05s (20Hz)")
        else:
            self.dt = 0.001  # 0.5ms for real hardware (2000Hz)
            self.get_logger().info("Using real hardware timing: dt=0.0005s (2000Hz)")

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

    def _check_robot_amount(self):
        """
        Robustly detect the number of robots by analyzing joint name prefixes from /joint_states.
        """
        self.get_logger().info("Waiting for /joint_states to detect robots...")

        # Wait for joint_states to be populated
        timeout = 10.0
        start = time.time()
        while not self.joint_states and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.joint_states:
            self.get_logger().error("No joint states received. Cannot detect robots.")
            rclpy.shutdown()
            sys.exit(1)

        # Extract unique robot prefixes based on joint naming patterns
        prefixes = set()
        for joint_name in self.joint_states.keys():
            if "/" in joint_name:
                # Multi-arm case: 'arm_right/shoulder_rotation' -> 'arm_right'
                prefix = joint_name.split("/")[0]
                prefixes.add(prefix)
            else:
                # Single arm case: 'shoulder_rotation' -> no prefix (None)
                prefixes.add("None")

        count = len(prefixes)

        # Log detected prefixes for debugging
        prefix_list = list(prefixes)
        self.get_logger().info(f"Detected {count} robot(s) with prefixes: {prefix_list}")

        if count > 2:
            self.get_logger().error(
                "More than 2 robots detected by joint name prefix. Only up to two are supported."
            )
            rclpy.shutdown()
            sys.exit(1)

        return count

    def _check_simulation_mode(self):
        """Detect if we're running in simulation or real hardware mode."""

        try:
            node_names = self.get_node_names()
            if "gz_ros_control" in node_names:
                return True
        except Exception as e:
            self.get_logger().debug(f"Could not check for gz_ros_control: {e}")

        return False

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

        if not msg.buttons[self.button_mapping["dead_man_switch"]]:
            return

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

    def _wait_for_controller_manager(self):
        self.get_logger().info("Waiting for controller_manager to be online...")
        while not self.controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "controller_manager service not available yet. Waiting...",
                throttle_duration_sec=10.0,
            )
        self.get_logger().info("controller_manager is online.")

    def run(self):
        self._wait_for_controller_manager()
        self._check_robot_amount()

    def wait_for_any_joint_trajectory_controller_active(self, timeout=60.0):
        """
        Wait until at least one controller with name starting with 'joint_trajectory_controller'
        is in state 'active'.
        """

        if not self.controller_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("controller_manager/list_controllers service not available!")
            raise RuntimeError("controller_manager/list_controllers service not available!")

        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            req = ListControllers.Request()
            future = self.controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None:
                for ctrl in future.result().controller:
                    self.get_logger().debug(f"Controller: {ctrl.name}, State: {ctrl.state}")
                    if ctrl.name.startswith("joint_trajectory_controller") and (
                        ctrl.state == "inactive" or ctrl.state == "active"
                    ):
                        self.get_logger().debug(f"Controller '{ctrl.name}' is inactive.")
                        return
            self.get_logger().debug("Waiting for a 'joint_trajectory_controller*' to be active...")
            time.sleep(0.5)
        self.get_logger().error("No active joint_trajectory_controller* after timeout!")
        raise TimeoutError("No active joint_trajectory_controller* after timeout.")


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
