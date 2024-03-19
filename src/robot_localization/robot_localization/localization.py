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

        self.raio = 
        self.distancia_rodas = 0.178
        self.pose = [0, 0, 0] # x, y, theta
        self.medidas = [0, 0] # esq, dir
        self.ultimas_medidas = [0, 0] # esq, dir
        self.distancias = [0, 0]

        self.estado_inicial = -4
        self.mapa = [-2.7, -0.7, 2.7] # posição central das três “portas” existentes
        self.pose[0] = estado_inicial # atualiza como estado_inicial a posição x de

        self.sigma_odometria = 0.2 # rad
        self.sigma_lidar = 0.175 # meters
        self.sigma_movimento = 0.002 # m

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        self.get_logger().info(str(self.ranges))
        
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
        self.get_logger().info(str(self.pose))

    def navigation(self):
        try:
            rclpy.spin_once(self)
            self.navigation_start()
            while(rclpy.ok):
                rclpy.spin_once(self)
                self.navigation_update()
        except (KeyboardInterrupt):
            pass

    def navigation_start(self):
      pass
      self.get_logger().info("Inicialize seu código aqui!")

    def navigation_update(self):
      pass
      self.get_logger().info("Aqui acontece o loop principal do seu código!")

def main(args=None):
     rclpy.init(args=args)

     navigator = Localization()

     try:
        rclpy.spin_once(self)
        self.navigation_start()
        while(rclpy.ok):
            rclpy.spin_once(self)
            self.navigation_update()
    except (KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()