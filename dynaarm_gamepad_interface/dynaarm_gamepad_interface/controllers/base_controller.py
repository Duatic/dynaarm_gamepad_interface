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

    def reset(self):
        """Reset controller state when switching back to this controller."""
        self.log_printed = False  # Reset logging state
