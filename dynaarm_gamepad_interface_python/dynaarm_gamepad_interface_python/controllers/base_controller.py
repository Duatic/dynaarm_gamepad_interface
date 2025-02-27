class BaseController:
    """Base class for all controllers, providing logging and common methods."""

    def __init__(self, node):
        self.node = node
        self.log_printed = False  # Track whether the log was printed

    def get_joint_states(self):
        """Retrieve latest joint positions from GamepadBase"""
        return self.node.joint_states

    def process_input(self, joy_msg):
        """Override this in child classes."""
        pass

    def reset_log(self):
        """Reset log state when the controller is switched."""
        self.log_printed = False

    def log_activation(self, message):
        """Log activation message once per activation."""
        if not self.log_printed:
            self.node.get_logger().info(message)
            self.log_printed = True
