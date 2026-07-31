import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/kinect/rgb', self.image_cb, 10)
        self.pixel_pub = self.create_publisher(PointStamped, '/detection/pixel', 10)
        self.target_pub = self.create_publisher(PointStamped, '/detection/target_coords', 10)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # 3D coords per unique label e.g. 'Pink_Circle_1'
        self.shape_3d_coords = {}
        self.create_subscription(PointStamped, '/detection/coords', self.coords_cb, 10)

        # Voice command: stores e.g. "circle_1"  (lowercase, underscore)
        self.voice_target = None
        self.create_subscription(String, '/voice_command', self.voice_cb, 10)

        self.get_logger().info('Detection node started')

    def coords_cb(self, msg):
        label = msg.header.frame_id
        self.shape_3d_coords[label] = (msg.point.x, msg.point.y, msg.point.z)

    def voice_cb(self, msg):
        """Receives e.g. 'circle_1' from voice node."""
        self.voice_target = msg.data.lower()
        self.get_logger().info(f"Voice target set: {self.voice_target}")

    def _label_matches_target(self, label: str) -> bool:
        """
        label  = 'Pink_Circle_1'   (Color_Shape_Number)
        target = 'circle_1'        (Shape_Number, lowercase)
        """
        if self.voice_target is None:
            return False
        parts = label.lower().split('_')   # ['pink', 'circle', '1']
        if len(parts) < 3:
            return False
        shape_number = f'{parts[1]}_{parts[2]}'   # 'circle_1'
        return shape_number == self.voice_target

    def detect_shape(self, contour):
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
        vertices = len(approx)
        if vertices == 3:
            return 'Triangle'
        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            ratio = float(w) / h
            return 'Square' if 0.9 <= ratio <= 1.1 else 'Rectangle'
        area = cv2.contourArea(contour)
        if peri == 0:
            return 'Unknown'
        circularity = 4 * np.pi * area / (peri ** 2)
        if circularity > 0.70:
            return 'Circle'
        return 'Unknown'

    def detect_color(self, hsv_roi):
        pink1 = cv2.inRange(hsv_roi, np.array([140, 30, 30]), np.array([180, 255, 255]))
        pink2 = cv2.inRange(hsv_roi, np.array([0,   30, 30]), np.array([10,  255, 255]))
        pink_count = cv2.countNonZero(cv2.bitwise_or(pink1, pink2))
        green = cv2.inRange(hsv_roi, np.array([35, 30, 30]), np.array([85, 255, 255]))
        green_count = cv2.countNonZero(green)
        if pink_count > green_count:
            return 'Pink'
        elif green_count > pink_count:
            return 'Green'
        return 'Unknown'

    def get_color_mask(self, hsv):
        pink1 = cv2.inRange(hsv, np.array([140, 30, 30]), np.array([180, 255, 255]))
        pink2 = cv2.inRange(hsv, np.array([0,   30, 30]), np.array([10,  255, 255]))
        pink_mask = cv2.bitwise_or(pink1, pink2)
        green_mask = cv2.inRange(hsv, np.array([35, 30, 30]), np.array([85, 255, 255]))
        return cv2.bitwise_or(pink_mask, green_mask)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        display = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_mask = self.get_color_mask(hsv)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  self.kernel, iterations=2)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        shape_counters = {}
        detected_shapes = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 2000 or area > 150000:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area == 0:
                continue
            if area / hull_area < 0.80:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect = float(w) / h
            if aspect < 0.3 or aspect > 3.5:
                continue
            shape = self.detect_shape(contour)
            if shape == 'Unknown':
                continue
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            hsv_roi = hsv[max(0, y):min(hsv.shape[0], y+h),
                          max(0, x):min(hsv.shape[1], x+w)]
            color = self.detect_color(hsv_roi)

            shape_counters[shape] = shape_counters.get(shape, 0) + 1
            number = shape_counters[shape]

            label = f'{color}_{shape}_{number}'
            display_label = f'{shape} {number}'
            is_target = self._label_matches_target(label)

            detected_shapes.append((label, shape, cx, cy, contour, is_target))

            # ── Box color: cyan highlight for voice target, green otherwise ──
            box_color = (0, 255, 255) if is_target else (0, 255, 0)
            thickness  = 3            if is_target else 2

            if is_target:
                # Semi-transparent highlight overlay
                overlay = display.copy()
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 255), -1)
                cv2.addWeighted(overlay, 0.15, display, 0.85, 0, display)

            cv2.rectangle(display, (x, y), (x + w, y + h), box_color, thickness)
            cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)

            cv2.putText(display, str(number),
                        (x + 5, y + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(display, display_label,
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255) if is_target else (0, 165, 255), 2)
            cv2.putText(display, color,
                        (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(display, f'px({cx},{cy})',
                        (x, y - 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 100), 1)

            if label in self.shape_3d_coords:
                wx, wy, wz = self.shape_3d_coords[label]
                coord_color = (0, 255, 255) if is_target else (0, 0, 255)
                cv2.putText(display, f'X:{wx:.3f} Y:{wy:.3f} Z:{wz:.3f}m',
                            (x, y + h + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5 if is_target else 0.4,
                            coord_color,
                            2 if is_target else 1)

                # ── Publish target 3D coords on dedicated topic ──
                if is_target:
                    pt = PointStamped()
                    pt.header.stamp = self.get_clock().now().to_msg()
                    pt.header.frame_id = label
                    pt.point.x = wx
                    pt.point.y = wy
                    pt.point.z = wz
                    self.target_pub.publish(pt)

            # Publish pixel coords for all shapes (for depth node)
            pt = PointStamped()
            pt.header.frame_id = label
            pt.point.x = float(cx)
            pt.point.y = float(cy)
            pt.point.z = 0.0
            self.pixel_pub.publish(pt)

        # ── Summary panel ──
        total = len(detected_shapes)
        cv2.putText(display, f'Detected: {total} objects',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Show active voice target at top
        if self.voice_target:
            cv2.putText(display, f'Target: {self.voice_target}',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        type_counts = {}
        for (label, shape, cx, cy, contour, _) in detected_shapes:
            type_counts[shape] = type_counts.get(shape, 0) + 1

        y_offset = 90 if self.voice_target else 58
        for shape_type, count in sorted(type_counts.items()):
            cv2.putText(display, f'  {count}x {shape_type}',
                        (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            y_offset += 22

        cv2.imshow('Detection', display)
        cv2.imshow('Color Mask', color_mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()