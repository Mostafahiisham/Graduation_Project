import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math

class OdometryFromEncoders(Node):
    def __init__(self):
        super().__init__('odometry_from_encoders')

        # Robot parameters (tune to match your robot)
        self.wheel_radius = 0.06  # meters
        self.wheel_base = 0.3     # meters

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_rpms',
            self.encoder_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

    def encoder_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        self.last_time = now

        # Assuming motor 1 = left, motor 2 = right
        try:
            rpm_left = msg.data[0]
            rpm_right = msg.data[1]
        except IndexError:
            self.get_logger().warn('Received incomplete RPM data')
            return

        # Convert RPM to linear velocity
        v_l = (2 * math.pi * self.wheel_radius * rpm_left) / 60.0
        v_r = (2 * math.pi * self.wheel_radius * rpm_right) / 60.0

        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheel_base

        # Integrate to update pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryFromEncoders()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
