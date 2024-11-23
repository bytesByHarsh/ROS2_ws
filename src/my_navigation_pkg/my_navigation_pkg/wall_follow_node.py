import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)


class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        # Timer for navigation logic
        self.timer = self.create_timer(0.1, self.control_loop)

        self.ranges = []


    def scan_callback(self, msg):
        self.ranges = msg.ranges
        # self.get_logger().info(f"Got : {len(self.ranges)}")

    def find_direction_range(self):
        left = self.ranges[45:136]
        front = self.ranges[315:] + self.ranges[0:45]
        right = self.ranges[225:315]

        return min(left), min(front), min(right)



    def control_loop(self):
        if not self.ranges:
            return

        twist = Twist()
        left, front, right = self.find_direction_range()

        if front < 0.5:
            self.get_logger().info("Turn fast to left")
            twist.linear.x = 0.05
            twist.angular.z = 0.5
        elif right > 0.2:
            self.get_logger().info("Find the wall")
            twist.linear.x = 0.1
            twist.angular.z = -0.1
        elif right < 0.2:
            self.get_logger().info("Move away from the wall")
            twist.linear.x = 0.05
            twist.angular.z = 0.05
        elif right > 0.2 and right < 0.3:
            self.get_logger().info("Follow the wall")
            twist.linear.x = 0.1
            twist.angular.z = 0
        elif left < 0.3:
            self.get_logger().info("Move away from obstacle")
            twist.linear.x = 0.0
            twist.angular.z = -0.1
        else:
            self.get_logger().info("Stop")
            twist.linear.x = 0
            twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    try:
        node = WallFollowingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stooping")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()