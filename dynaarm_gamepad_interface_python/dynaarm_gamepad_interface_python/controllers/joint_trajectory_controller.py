from .base_controller import BaseController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
from collections import deque

class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    def __init__(self, node):
        super().__init__(node)

        # Publisher for sending joint trajectory commands
        self.joint_trajectory_publisher = self.node.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.is_joystick_idle = True  # Track joystick idle state
        self.initial_positions_set = False  # Flag to check if initial positions are set
        self.commanded_positions = []  # Stores the current commanded positions
        self.active_axes = {}  # Track active joystick axes (to detect when they go to 0)
        self.last_command_time = time.time()  # Track last command time

        # FIFO-Warteschlange für sanfte Bewegungen (mehr Punkte, aber kontrollierte Verzögerung)
        self.trajectory_buffer = deque(maxlen=15)

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
        displacement_scale = 0.004  # Noch kleinere Schrittgröße für feinfühlige Steuerung
        deadzone = 0.08  # Kleinere Deadzone, um feine Bewegungen zuzulassen
        max_step = 0.008  # Kleinere maximale Schrittweite

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

            # Wenn die Achse inaktiv wurde, setze das Ziel auf die aktuelle Position
            if abs(axis_val) < deadzone and self.active_axes.get(i, False):      
                joint_states = self.get_joint_states()    
                current_positions = list(joint_states.values())
                self.commanded_positions[i] = current_positions[i]

            # Update command wenn die Achse aktiv ist, aber mit max_step-Limit
            if abs(axis_val) > deadzone:
                target_position = self.commanded_positions[i] + axis_val * displacement_scale
                delta = target_position - self.commanded_positions[i]
                if abs(delta) > max_step:
                    delta = max_step * math.copysign(1, delta)  # Begrenze Änderung auf max_step
                self.commanded_positions[i] += delta

                any_axis_active = True
                self.active_axes[i] = True  # Mark axis as active
            else:
                self.active_axes[i] = False  # Mark axis as inactive

        # **FIFO-Warteschlange verwenden für flüssige Bewegung**
        self.trajectory_buffer.append(list(self.commanded_positions))

        if any_axis_active:
            self.is_joystick_idle = False
            if len(self.trajectory_buffer) == self.trajectory_buffer.maxlen:
                self.publish_joint_trajectory(list(self.trajectory_buffer), speed_percentage=85.0)  # Höhere Rate
            self.last_command_time = time.time()
        elif not any_axis_active and not self.is_joystick_idle:
            if time.time() - self.last_command_time > 0.05:  # Sehr kurze Verzögerung
                self.publish_joint_trajectory(list(self.trajectory_buffer), speed_percentage=85.0)
                self.is_joystick_idle = True
                # **Letztes stabilisierendes Signal senden**
                self.trajectory_buffer.appendleft(self.commanded_positions[:])
                self.publish_joint_trajectory(list(self.trajectory_buffer), speed_percentage=100.0)
        elif not any_axis_active and self.is_joystick_idle:
            self.last_command_time = time.time()

    def publish_joint_trajectory(self, trajectory_points, speed_percentage):
        """Publishes a joint trajectory message with multiple points for smoother movement."""
        joint_names = list(self.get_joint_states().keys())

        if not joint_names:
            self.node.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not trajectory_points:
            self.node.get_logger().error("No trajectory points available to publish.")
            return

        # Clamp speed percentage between 1 and 100
        speed_percentage = max(1.0, min(100.0, speed_percentage))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        # Erzeuge eine Trajektorie mit mehreren Punkten
        for i, target_positions in enumerate(trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = target_positions

            # Zeitintervall zwischen Punkten anpassen (kleine Werte für schnelles Updaten)
            time_seconds = (40.0 / speed_percentage) * (i + 1) / len(trajectory_points)
            point.time_from_start.sec = int(math.floor(time_seconds))
            point.time_from_start.nanosec = int((time_seconds - point.time_from_start.sec) * 1e9)

            trajectory_msg.points.append(point)

        self.joint_trajectory_publisher.publish(trajectory_msg)
        self.node.get_logger().debug(f"Published Smooth Joint Trajectory: {trajectory_points[-1]}")
