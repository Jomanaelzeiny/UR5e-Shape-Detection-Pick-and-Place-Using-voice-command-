import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import os
import numpy as np

CALIB_FILE = os.path.expanduser('~/.ros/camera_info/kinect_calibration.npz')

DEFAULT_FX = 525.0
DEFAULT_FY = 525.0
DEFAULT_CX = 319.5
DEFAULT_CY = 239.5


class CoordTransformNode(Node):
    def __init__(self):
        super().__init__('coord_transform_node')

        if os.path.exists(CALIB_FILE):
            calib   = np.load(CALIB_FILE)
            K       = calib['K_l']
            self.fx = float(K[0, 0])
            self.fy = float(K[1, 1])
            self.cx = float(K[0, 2])
            self.cy = float(K[1, 2])
            self.get_logger().info(f'Loaded calibration from {CALIB_FILE}')
        else:
            self.fx, self.fy = DEFAULT_FX, DEFAULT_FY
            self.cx, self.cy = DEFAULT_CX, DEFAULT_CY
            self.get_logger().warn('No calibration file — using default intrinsics')

        self.create_subscription(
            PointStamped, '/detection/pixel_with_depth', self.pixel_cb, 10)
        self.coords_pub = self.create_publisher(
            PointStamped, '/detection/coords', 10)

        self.get_logger().info('Coord transform node started')

    def pixel_cb(self, msg):
        shape_name = msg.header.frame_id  # carry shape name through
        u = msg.point.x
        v = msg.point.y
        Z = msg.point.z

        if Z <= 0.0:
            self.get_logger().warn(
                'Zero/negative depth, skipping.', throttle_duration_sec=2)
            return

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        out = PointStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = shape_name  # pass shape name to commander
        out.point.x = X
        out.point.y = Y
        out.point.z = Z
        self.coords_pub.publish(out)

        self.get_logger().info(
            f'[{shape_name}] 3D coords → X={X:.3f} m  Y={Y:.3f} m  Z={Z:.3f} m',
            throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()