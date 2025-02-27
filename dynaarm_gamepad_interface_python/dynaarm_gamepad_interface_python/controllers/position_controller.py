from .base_controller import BaseController
from std_msgs.msg import Float64MultiArray

class PositionController(BaseController):
    """Handles position control using the gamepad"""

    def __init__(self, node):
        super().__init__(node)

        # Publisher for sending position commands
        self.position_command_publisher = self.node.create_publisher(
            Float64MultiArray, "/position_controller/commands", 10
        )

        self.is_joystick_idle = True  # Track joystick idle state
        self.initial_positions_set = False  # Flag to check if initial positions are set
        self.commanded_positions = []  # Stores the current commanded positions

    def process_input(self, joy_msg):
        """Processes joystick input and updates joint positions."""
        super().process_input(joy_msg)  # Base logging logic

        joint_states = self.get_joint_states()
        if not joint_states:
            self.node.get_logger().warn("No joint states available. Cannot process input.", throttle_duration_sec=5.0)
            return

        joint_names = list(joint_states.keys())  # Extract joint names
        current_positions = list(joint_states.values())  # Extract joint positions

        # Initialize commanded_positions with current positions on first run
        if not self.initial_positions_set:
            self.commanded_positions = current_positions[:]
            self.initial_positions_set = True
            self.node.get_logger().info("Initialized commanded positions with current joint states.")

        any_axis_active = False
        displacement_scale = 0.005  # Scale for joystick movement sensitivity
        deadzone = 0.1  # Define a small deadzone to ignore noise

        # Process each joint based on gamepad axis input
        for i, joint_name in enumerate(joint_names):
            axis_val = 0.0

            # Map axes to specific joints
            if i == 0 and len(joy_msg.axes) > self.node.axis_mapping["left_joystick"]["x"]:
                axis_val = joy_msg.axes[self.node.axis_mapping["left_joystick"]["x"]]
            elif i == 1 and len(joy_msg.axes) > self.node.axis_mapping["left_joystick"]["y"]:
                axis_val = joy_msg.axes[self.node.axis_mapping["left_joystick"]["y"]]
            elif i == 2 and len(joy_msg.axes) > self.node.axis_mapping["right_joystick"]["y"]:
                axis_val = joy_msg.axes[self.node.axis_mapping["right_joystick"]["y"]]
            elif i == 3 and len(joy_msg.axes) > self.node.axis_mapping["right_joystick"]["x"]:
                axis_val = joy_msg.axes[self.node.axis_mapping["right_joystick"]["x"]]
            elif i == 4:  # Trigger-based control
                left_trigger = joy_msg.axes[self.node.axis_mapping["triggers"]["left"]]
                right_trigger = joy_msg.axes[self.node.axis_mapping["triggers"]["right"]]
                axis_val = right_trigger - left_trigger

            # Update command if axis is active
            if abs(axis_val) > deadzone:  # Deadzone check
                self.commanded_positions[i] += axis_val * displacement_scale  # Apply offset
                any_axis_active = True

        # Publish position command if movement detected
        if any_axis_active:
            self.is_joystick_idle = False
            self.publish_position_command(self.commanded_positions)

        # If joystick was just released, hold position once
        elif not any_axis_active and not self.is_joystick_idle:
            self.publish_position_command(self.commanded_positions)  # Hold position
            self.is_joystick_idle = True  # Mark as idle

    def publish_position_command(self, target_positions):
        """Publishes a position command message."""
        if not target_positions:
            self.node.get_logger().error("No target positions available. Cannot publish command.")
            return

        command_msg = Float64MultiArray()
        command_msg.data = target_positions

        self.position_command_publisher.publish(command_msg)
        self.node.get_logger().debug(f"Published Position Command: {target_positions}")
