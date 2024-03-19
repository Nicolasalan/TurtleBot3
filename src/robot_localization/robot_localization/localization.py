import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class Localization(Node):

    def __init__(self):
        super().__init__('localization')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        self.get_logger().info(str(self.laser))

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
        self.get_logger().info(str(self.pose))

    def navigation_start(self):
      self.get_logger().info("Inicialize seu código aqui!")

    def navigation_update(self):
      self.get_logger().info("Aqui acontece o loop principal do seu código!")

def main(args=None):
    rclpy.init(args=args)

    navigator = Localization()

    try:
        rclpy.spin_once(navigator)
        navigator.navigation_start()
        while(rclpy.ok):
            rclpy.spin_once(navigator)
            navigator.navigation_update()
    except (KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()
