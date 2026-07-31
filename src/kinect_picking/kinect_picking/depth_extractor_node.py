import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np


class DepthExtractorNode(Node):
    def __init__(self):
        super().__init__('depth_extractor_node')
        self.bridge    = CvBridge()
        self.depth_map = None

        self.create_subscription(Image,        '/kinect/depth',    self.depth_cb, 10)
        self.create_subscription(PointStamped, '/detection/pixel', self.pixel_cb, 10)
        self.depth_pub = self.create_publisher(PointStamped, '/detection/pixel_with_depth', 10)
        self.get_logger().info('Depth extractor node started')

    def depth_cb(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def pixel_cb(self, msg):
        if self.depth_map is None:
            self.get_logger().warn('No depth map yet', throttle_duration_sec=2)
            return

        u = int(msg.point.x)
        v = int(msg.point.y)
        shape_name = msg.header.frame_id  # carry shape name through

        h, w = self.depth_map.shape
        r = 35  # increased from 15
        patch = self.depth_map[
            max(0, v - r): min(h, v + r),
            max(0, u - r): min(w, u + r)
        ]

        valid = patch[(patch > 0.4) & (patch < 8.0)]
        if len(valid) < 3:
            self.get_logger().warn(
                f'Not enough valid depth pixels at ({u},{v})',
                throttle_duration_sec=2)
            return

        depth = float(np.median(valid))

        out = PointStamped()
        out.header.frame_id = shape_name  # pass shape name forward
        out.point.x = msg.point.x
        out.point.y = msg.point.y
        out.point.z = depth
        self.depth_pub.publish(out)

        self.get_logger().info(
            f'[{shape_name}] Depth at ({u},{v}) = {depth:.3f} m',
            throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()