from dynaarm_gamepad_interface.controllers.base_controller import BaseController

class FreedriveController(BaseController):
    """Handles freedrive mode."""

    def process_input(self, joy_msg):
        self.log_activation("Move the arm freely. No input is needed from the gamepad.")
