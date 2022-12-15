import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener


class Broadcast(Node):
    """Publishes a static transform between the camera frame and panda_link0 from a yaml."""

    def __init__(self):
        """Set up a broadcaster, retrieve parameters from a yaml, and initialize variables."""
        super().__init__('broadcast')
        # Import parameters from tf.yaml
        self.declare_parameters(
            namespace='',
            parameters=[
                ('w_rot', 0.0),
                ('x_rot', 0.0),
                ('x_trans', 0.0),
                ('y_rot', 0.0),
                ('y_trans', 0.0),
                ('z_rot', 0.0),
                ('z_trans', 0.0)
            ])

        # Assign parameter values
        self.rw = self.get_parameter("w_rot").get_parameter_value().double_value
        self.rx = self.get_parameter("x_rot").get_parameter_value().double_value
        self.ry = self.get_parameter("y_rot").get_parameter_value().double_value
        self.rz = self.get_parameter("z_rot").get_parameter_value().double_value
        self.tx = self.get_parameter("x_trans").get_parameter_value().double_value
        self.ty = self.get_parameter("y_trans").get_parameter_value().double_value
        self.tz = self.get_parameter("z_trans").get_parameter_value().double_value

        # Set timer frequence
        self.freq = 60.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define frames
        self.frame_camera = "camera_link"
        self.frame_base = "panda_link0"
        self.cam_to_base = TransformStamped()

    def timer_callback(self):
        """Continuously published the transform between the camera and panda_link0."""
        # create tf between camera and panda_link0
        self.cam_to_base.header.stamp = self.get_clock().now().to_msg()
        self.cam_to_base.header.frame_id = self.frame_camera
        self.cam_to_base.child_frame_id = self.frame_base
        self.cam_to_base.transform.translation.x = self.tx
        self.cam_to_base.transform.translation.y = self.ty
        self.cam_to_base.transform.translation.z = self.tz
        self.cam_to_base.transform.rotation.x = self.rx
        self.cam_to_base.transform.rotation.y = self.ry
        self.cam_to_base.transform.rotation.z = self.rz
        self.cam_to_base.transform.rotation.w = self.rw
        self.tf_broadcaster.sendTransform(self.cam_to_base)


def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    b = Broadcast()
    rclpy.spin(b)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
