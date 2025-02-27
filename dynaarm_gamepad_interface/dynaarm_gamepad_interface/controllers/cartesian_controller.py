from dynaarm_gamepad_interface.controllers.base_controller import BaseController


class CartesianController(BaseController):
    """Handles Cartesian control mode."""

    def process_input(self, joy_msg):
        self.log_activation("Processing Cartesian control input...")
        # Add movement logic here
