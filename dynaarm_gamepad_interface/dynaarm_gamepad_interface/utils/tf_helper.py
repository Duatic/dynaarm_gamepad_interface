import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class TFHelper:

    def __init__(self, node):
        """Initializes the URDF helper and loads the Pinocchio model."""
        self.node = node

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def compute_fk(self, base_frame, ee_frame):
        """Computes the forward kinematics using Pinocchio."""

        try:
            self.node.get_logger().debug(f"Looking up transform: {base_frame} -> {ee_frame}")
            transform = self.tf_buffer.lookup_transform(base_frame, ee_frame, rclpy.time.Time())

            # Convert TF2 transform to PoseStamped
            pose = PoseStamped()
            pose.header = transform.header

            pose.pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z,
            )

            pose.pose.orientation = Quaternion(
                x=transform.transform.rotation.x,
                y=transform.transform.rotation.y,
                z=transform.transform.rotation.z,
                w=transform.transform.rotation.w,
            )

            return pose

        except tf2_ros.LookupException as e:
            self.node.get_logger().error(f"TF2 lookup failed: {e}")
