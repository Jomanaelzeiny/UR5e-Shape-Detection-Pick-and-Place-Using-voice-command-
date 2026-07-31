import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import freenect
import numpy as np
import cv2
import time
from rclpy.executors import MultiThreadedExecutor


class KinectNode(Node):

    def __init__(self):
        super().__init__('kinect_node')
        self.bridge = CvBridge()

        self.rgb_pub   = self.create_publisher(Image, '/kinect/rgb',   10)
        self.depth_pub = self.create_publisher(Image, '/kinect/depth', 10)

        self._fail_count = 0
        self._max_fails  = 10

        self.create_timer(1.0 / 15.0, self.publish_frames)  # 15 Hz — safer than 30

        self.get_logger().info('Kinect 360 node started (freenect)')

    def publish_frames(self):
        try:
            # ---------- RGB ----------
            result = freenect.sync_get_video()
            if result is not None:
                rgb_raw, _ = result
                if rgb_raw is not None:
                    bgr = cv2.cvtColor(rgb_raw, cv2.COLOR_RGB2BGR)
                    rgb_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                    rgb_msg.header.stamp    = self.get_clock().now().to_msg()
                    rgb_msg.header.frame_id = 'kinect_rgb'
                    self.rgb_pub.publish(rgb_msg)

            # ---------- Depth ----------
            result = freenect.sync_get_depth()
            if result is not None:
                depth_raw, _ = result
                if depth_raw is not None:
                    depth_f = depth_raw.astype(np.float32)

                    with np.errstate(divide='ignore', invalid='ignore'):
                        depth_m = 1.0 / (depth_f * -0.0030711016 + 3.3309495161)

                    depth_m[depth_raw >= 2047] = 0.0
                    depth_m[depth_m < 0.4]     = 0.0
                    depth_m[depth_m > 8.0]     = 0.0

                    depth_msg = self.bridge.cv2_to_imgmsg(
                        depth_m.astype(np.float32), encoding='32FC1')
                    depth_msg.header.stamp    = self.get_clock().now().to_msg()
                    depth_msg.header.frame_id = 'kinect_depth'
                    self.depth_pub.publish(depth_msg)

            self._fail_count = 0  # reset on success

        except Exception as e:
            self._fail_count += 1
            self.get_logger().warn(
                f'Kinect read error ({self._fail_count}/{self._max_fails}): {e}',
                throttle_duration_sec=2)

            if self._fail_count >= self._max_fails:
                self.get_logger().error(
                    'Too many Kinect failures. Waiting 3 seconds before retry...')
                time.sleep(3)
                self._fail_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = KinectNode()  # your node class name
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
