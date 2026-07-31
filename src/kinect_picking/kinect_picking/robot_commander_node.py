import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class RobotCommanderNode(Node):
    """
    Subscribes to /detection/coords  (PointStamped, camera-relative metres)

    Sends pick commands to the UR5e robot.

    NOTE: The camera-to-robot transform (T_robot_camera) must be set via the
    class attributes below once you have measured / calibrated the physical
    offset between the Kinect and the robot base frame.

    Currently the node prints the received camera-frame coordinates and
    publishes a string command.  Replace the send_pick_command() body with
    your actual UR5e driver calls (e.g. ur_rtde, MoveIt2 action client, etc.)
    """

    # -------------------------------------------------------------------
    # Camera → Robot base frame translation offsets (metres).
    # Measure these once the Kinect is mounted on the robot cell.
    # These are PLACEHOLDERS — update them after physical measurement.
    # -------------------------------------------------------------------
    CAM_OFFSET_X = 0.0   # camera origin X relative to robot base
    CAM_OFFSET_Y = 0.0   # camera origin Y relative to robot base
    CAM_OFFSET_Z = 0.0   # camera origin Z relative to robot base

    def __init__(self):
        super().__init__('robot_commander_node')

        self.create_subscription(
            PointStamped, '/detection/coords', self.coords_cb, 10)

        self.cmd_pub = self.create_publisher(String, '/robot/command', 10)

        self.get_logger().info('✅ Robot commander node started')
        self.get_logger().info('📥 /detection/coords')
        self.get_logger().info('📤 /robot/command')

    # ------------------------------------------------------------------ #
    def coords_cb(self, msg):
        # Camera-relative coordinates
        cam_x = msg.point.x
        cam_y = msg.point.y
        cam_z = msg.point.z

        # Simple translation to robot base frame
        # (replace with full rotation matrix if camera is tilted)
        robot_x = cam_x + self.CAM_OFFSET_X
        robot_y = cam_y + self.CAM_OFFSET_Y
        robot_z = cam_z + self.CAM_OFFSET_Z

        self.get_logger().info(
            f'Camera frame  → X={cam_x:.3f}  Y={cam_y:.3f}  Z={cam_z:.3f}')
        self.get_logger().info(
            f'Robot frame   → X={robot_x:.3f}  Y={robot_y:.3f}  Z={robot_z:.3f}')

        self.send_pick_command(robot_x, robot_y, robot_z)

    # ------------------------------------------------------------------ #
    def send_pick_command(self, x, y, z):
        """
        Replace this with your actual UR5e movement call.
        Example placeholders shown below.
        """
        cmd      = String()
        cmd.data = f'PICK {x:.4f} {y:.4f} {z:.4f}'
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f'📡 Sent pick command: {cmd.data}')

        # --- Future: MoveIt2 / ur_rtde integration goes here ---
        # goal = MoveToPoint.Goal()
        # goal.target.x = x
        # goal.target.y = y
        # goal.target.z = z
        # self._move_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
