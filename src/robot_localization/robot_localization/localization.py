import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from math import exp, sqrt, pi
from math import cos, sin
import numpy as np

Inf = float('inf')

class Localization(Node):

    def __init__(self):
        super().__init__('localization')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.laser = []
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.position = 0
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.joint = [0, 0]
        self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.raio = 0.033
        self.distancia_rodas = 0.178
        self.pose = [0, 0, 0]
        self.medidas = [0, 0]
        self.ultimas_medidas = [0, 0]
        self.distancias = [0, 0]

        self.estado_inicial = -1.999
        self.mapa = 1.07810
        self.pose[0] = self.estado_inicial

        self.sigma_odometria = 0.2 # rad
        self.sigma_lidar = 0.175 # meters
        self.sigma_movimento = 0.002 # m

    def gaussian(self, x, mean, sigma):
        return (1 / (sigma*sqrt(2*pi))) * exp(-((x-mean)**2) / (2*sigma**2))

    def update(self):
        self.medidas[0] = self.joint[0]
        self.medidas[1] = self.joint[1]
        diff = self.medidas[0] - self.ultimas_medidas[0] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[0] = diff * self.raio + np.random.normal(0,0.002) # determina distância percorrida em metros e adiciona um
        self.ultimas_medidas[0] = self.medidas[0]
        diff = self.medidas[1] - self.ultimas_medidas[1] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[1] = diff * self.raio + np.random.normal(0,0.002) # determina distância percorrida em metros + pequeno erro
        self.ultimas_medidas[1] = self.medidas[1]
        # ## cálculo da dist linear e angular percorrida no timestep
        deltaS = (self.distancias[0] + self.distancias[1]) / 2.0
        deltaTheta = (self.distancias[1] - self.distancias[0]) / self.distancia_rodas
        self.pose[2] = (self.pose[2] + deltaTheta) % 6.28 # atualiza o valor Theta (diferença da divisão por 2π)
        # decomposição x e y baseado no ângulo
        deltaSx = deltaS * cos(self.pose[2])
        deltaSy = deltaS * sin(self.pose[2])
        # atualização acumulativa da posição x e y
        self.pose[0] = self.pose[0] + deltaSx # atualiza x
        self.pose[1] = self.pose[1] + deltaSy # atualiza y

    def joint_callback(self, msg):
        self.joint = msg.position
        #self.get_logger().info(str(self.joint))

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        #self.get_logger().info(str(self.laser))

    def listener_callback_odom(self, msg):
        self.position = msg.pose.pose
        #self.get_logger().info(str(self.pose))

    def navigation_start(self):
        pass
        #self.get_logger().info("Inicialize seu código aqui!")

    def navigation_update(self):
      #self.get_logger().info("Aqui acontece o loop principal do seu código!")
      controle = 0
      cont = 0
      porta = 0
      leitura = self.laser

      self.update() # atualiza a nova pose do robô
      if len(leitura) == 0:
          pass
      else:
        if controle == 1:
            self.sigma_movimento = self.sigma_movimento + 0.002
        if leitura[72] == Inf and leitura[108] == Inf:
            media_nova = (self.mapa * self.sigma_movimento + self.pose[0]*self.sigma_lidar) / (self.sigma_movimento+self.sigma_lidar)
            sigma_novo = 1 / (1/self.sigma_movimento + 1/self.sigma_lidar)

            self.pose[0] = media_nova # a nova posição x do robô
            sigma_movimento = sigma_novo

            if porta == 0: porta = 1 # altera para a próxima porta 0 → 1 ; 1 → 2
            elif porta == 1: porta = 2
      cont += 1

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
