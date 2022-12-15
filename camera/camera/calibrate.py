import rclpy
import math
import yaml
import numpy as np
from enum import Enum, auto
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_path


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration.
    """

    LISTEN = auto(),
    CALIBRATE = auto(),
    WRITE = auto()
    DONE = auto()


def quaternion_from_euler(ai, aj, ak):  # I STOLE THIS FROM THE ROS DOCS
    """
    Change euler corrdinates into a quaternian.

    Input
    :param ai: a roll value
    :param aj: a pitch value
    :param aj: a yaw value

    Output
    :quaternian: a quaternian (x, y, z, w)
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def quaternion_multiply(Q0, Q1):  # I STOLE THIS FROM AUTOMATIC ADDISON
    """
    Multiplies two quaternions.

    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31)
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]

    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]

    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


def deg_to_rad(deg):
    """
    Change degrees to radians.

    Input
    :deg: An integer value of degrees

    Output
    :rad: An integer value of radians
    """
    rad = math.pi/180*deg
    return rad


class Calibrate(Node):
    """
    Run a calibration for the robot using april tags and the tf tree.

    Listen to the existing transformations between the ccamera and the tag and the end effector and
    the robot. Use the realtions to create a tf between the camera in the base so that objects in
    the camera's view can be found by the robot.
    """

    def __init__(self):
        """Initialize variables and set up a broadcaster."""
        super().__init__('cali')
        self.freq = 60.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define frames
        self.frame_camera = "camera_color_optical_frame"
        self.frame_tag = "tag36h11:0"
        self.frame_rotate = "rotate"
        self.frame_ee = "panda_hand_tcp"
        self.frame_base = "panda_link0"

        # initalize variables
        self.rot = TransformStamped()
        self.t = TransformStamped()
        self.rot_base = TransformStamped()
        self.state = State.LISTEN
        self.listen = 0

        # variables to average the transform
        self.count = 0
        self.avg_trans_x = []
        self.avg_trans_y = []
        self.avg_trans_z = []
        self.avg_rot_x = []
        self.avg_rot_y = []
        self.avg_rot_z = []
        self.avg_rot_w = []

    def timer_callback(self):
        """
        Use existing transforms to create a transform between panda_link0 and the camera.

        Use listeners to get transforms then utilizing quaternion operations get the appropriate
        transform from the camera to panda_link0 and write it to a yaml for future use.
        """
        if self.state == State.LISTEN:
            # listener for the camera to tag
            try:
                r = self.tf_buffer.lookup_transform(
                    self.frame_camera,
                    self.frame_tag,
                    rclpy.time.Time())
                self.og_q = np.array([r.transform.rotation.x, r.transform.rotation.y,
                                      r.transform.rotation.z, r.transform.rotation.w])
            except TransformException:
                self.get_logger().info(
                    f'Could not transform {self.frame_camera} to {self.frame_tag}')
                return

            # listener for the end effector to the base
            try:
                self.s = self.tf_buffer.lookup_transform(
                    self.frame_ee,
                    self.frame_base,
                    rclpy.time.Time())
            except TransformException:
                self.get_logger().info(
                    f'Could not transform {self.frame_ee} to {self.frame_base}')
                return
            if self.listen < 300:
                # Wait for all the frames to be published
                self.listen += 1
            else:
                self.state = State.CALIBRATE

        if self.state == State.CALIBRATE:
            rad = deg_to_rad(90)
            # Create tf between the tag and the rotated frame
            self.rot.header.stamp = self.get_clock().now().to_msg()
            self.rot.header.frame_id = self.frame_tag
            self.rot.child_frame_id = self.frame_rotate
            self.rot.transform.translation.x = 0.0
            self.rot.transform.translation.y = 0.0
            self.rot.transform.translation.z = 0.0
            # Get the quaternian between the tag and the end effector
            # Multiply the tag by the ende effector quaternian
            z_rot = quaternion_from_euler(0.0, 0.0, rad)
            qz_rot = quaternion_multiply(self.og_q, z_rot)
            y_rot = quaternion_from_euler(-rad, 0.0, 0.0)
            q_final = quaternion_multiply(qz_rot, y_rot)
            self.rot.transform.rotation.x = q_final[0]
            self.rot.transform.rotation.y = q_final[1]
            self.rot.transform.rotation.z = q_final[2]
            self.rot.transform.rotation.w = q_final[3]
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(self.rot)

            # create a tf between the rotated frame and panda_link0
            self.rot_base = self.s
            self.rot_base.header.stamp = self.get_clock().now().to_msg()
            self.rot_base.header.frame_id = self.frame_rotate
            self.rot_base.child_frame_id = self.frame_base
            self.tf_broadcaster.sendTransform(self.rot_base)

            # listener for the camera to base
            try:
                cam_base = self.tf_buffer.lookup_transform(
                    self.frame_camera,
                    self.frame_base,
                    rclpy.time.Time())
                # Average the values of the transformation for the translations and rotations over
                # 200 frames
                if self.count < 200:
                    self.get_logger().info(f"COUNT:{self.count}")
                    self.avg_trans_x.append(cam_base.transform.translation.x)
                    self.avg_trans_y.append(cam_base.transform.translation.y)
                    self.avg_trans_z.append(cam_base.transform.translation.z)
                    self.avg_rot_x.append(cam_base.transform.rotation.x)
                    self.avg_rot_y.append(cam_base.transform.rotation.y)
                    self.avg_rot_z.append(cam_base.transform.rotation.z)
                    self.avg_rot_w.append(cam_base.transform.rotation.w)
                    self.count += 1
                else:
                    # Write the transform parameters to a yaml for later use without calibrating
                    self.dump = {'/**': {'ros__parameters':
                                         {'x_trans': float(np.mean(self.avg_trans_x)),
                                          'y_trans': float(np.mean(self.avg_trans_y)),
                                          'z_trans': float(np.mean(self.avg_trans_z)),
                                          'x_rot': float(np.mean(self.avg_rot_x)),
                                          'y_rot': float(np.mean(self.avg_rot_y)),
                                          'z_rot': float(np.mean(self.avg_rot_z)),
                                          'w_rot': float(np.mean(self.avg_rot_w))}}}
                    self.state = State.WRITE
            except TransformException:
                self.get_logger().info(
                    f'Could not transform {self.frame_camera} to {self.frame_base}')
                return

        if self.state == State.WRITE:
            # Write the transform information to the tf.yaml in the share directory
            # Must be done each time the robot is being reset
            camera_path = get_package_share_path('camera')
            tf_path = str(camera_path)+'/tf.yaml'
            with open(str(tf_path), 'w') as outfile:
                outfile.write(yaml.dump(self.dump, default_flow_style=False))
            self.state = State.DONE

        if self.state == State.DONE:
            # Print message when done calibrating
            self.get_logger().info("Done calibrating!")


def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    c = Calibrate()
    rclpy.spin(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
