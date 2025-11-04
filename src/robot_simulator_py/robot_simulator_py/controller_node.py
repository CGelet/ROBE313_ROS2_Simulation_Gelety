import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF buffer + listener to read odom -> base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State machine variables
        self.state = 'INIT'
        self.side_index = 0              # 0..3
        self.side_length = 2.0           # meters
        self.lin_speed = 0.2             # m/s
        self.ang_speed = 0.3             # rad/s

        self.pos_tol = 0.02              # m
        self.angle_tol = math.radians(.1) # rad

        self.start_x = 0.0
        self.start_y = 0.0
        self.turn_target_yaw = 0.0

        self.get_logger().info('controller_node started')

    # ---------- helpers ----------

    @staticmethod
    def yaw_from_quaternion(q):
        # standard yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        # Wrap to [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def ang_diff(a, b):
        # shortest angular difference a - b in [-pi, pi]
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def stop_cmd(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    # ---------- main control loop ----------

    def control_loop(self):
        # Get current pose from tf: odom -> base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # TF not ready yet
            self.get_logger().debug('Waiting for TF odom->base_link...')
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = self.yaw_from_quaternion(q)

        cmd = Twist()

        # ----- state machine -----
        if self.state == 'INIT':
            # Record starting position for first side
            self.start_x = x
            self.start_y = y
            self.state = 'DRIVE'
            self.get_logger().info('State: DRIVE, side 0')

        elif self.state == 'DRIVE':
            # Distance traveled along current side
            dx = x - self.start_x
            dy = y - self.start_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= self.side_length - self.pos_tol:
                # Side done, prepare to turn
                self.stop_cmd()
                self.state = 'TURN_INIT'
                self.get_logger().info(f'Finished side {self.side_index}, preparing to turn')
            else:
                cmd.linear.x = self.lin_speed
                cmd.angular.z = 0.0

        elif self.state == 'TURN_INIT':
            # Compute target yaw = yaw + 90 degrees
            target = yaw + math.pi / 2.0
            self.turn_target_yaw = self.wrap_angle(target)
            self.state = 'TURN'
            self.get_logger().info(
                f'State: TURN toward yaw {self.turn_target_yaw:.2f} rad')

        elif self.state == 'TURN':
            err = self.ang_diff(self.turn_target_yaw, yaw)

            if abs(err) < self.angle_tol:
                # Turn complete
                self.stop_cmd()
                self.side_index += 1

                if self.side_index >= 4:
                    self.state = 'STOP'
                    self.get_logger().info('Completed square, stopping')
                else:
                    # Prepare next straight segment
                    self.start_x = x
                    self.start_y = y
                    self.state = 'DRIVE'
                    self.get_logger().info(
                        f'State: DRIVE, side {self.side_index}')
            else:
                # Turn in direction of error
                cmd.angular.z = self.ang_speed if err > 0.0 else -self.ang_speed
                cmd.linear.x = 0.0

        elif self.state == 'STOP':
            # Just ensure we keep sending zero velocity
            cmd = Twist()

        # Publish command (for all states except early TF failures)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
