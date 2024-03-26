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

from matplotlib import pyplot as plt

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

        self.cmd = 0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.raio = 0.033
        self.distancia_rodas = 0.178
        self.pose = [0, 0, 0]
        self.medidas = [0, 0]
        self.ultimas_medidas = [0, 0]
        self.distancias = [0, 0]

        self.estado_inicial = -1.999
        self.mapa = [-1.66, -0.561, 0.569]
        self.pose[0] = self.estado_inicial

        self.sigma_odometria = 0.2 # rad
        self.sigma_lidar = 0.175 # meters
        self.sigma_movimento = 0.002 # m

        self.porta = 0
        self.controle = 0

        self.x = np.linspace(-4.5, 4.5, 500) # cria um vetor x de 500 valores entre -4.5 e 4.5
        self.y = np.zeros(500) # cria um vetor y de 500 valores zeros
        self.y2 = np.zeros(500)
        self.y3 = np.zeros(500)
        self.fig, self.ax = plt.subplots()

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

    def cmd_vel_callback(self, msg):
        self.cmd = msg.linear.x
        #self.get_logger().info(str(self.joint))

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        #self.get_logger().info(str(self.laser))

    def listener_callback_odom(self, msg):
        self.position = msg.pose.pose
        #self.get_logger().info(str(self.pose))

    def navigation_start(self):
        self.get_logger().info("Inicialize seu código aqui!")

    def navigation_update(self):
      #self.get_logger().info("Aqui acontece o loop principal do seu código!")
      leitura = self.laser
      cont = 0

      self.update() # atualiza a nova pose do robô

      print("Sigma_movimento: ", self.sigma_movimento)
      print("Pose: ", self.pose[0])

      if cont % 4 == 0: # a cada 4 passos, plotar em preto “b” a gaussiana da posição do robô em
        for i in range(len(self.x)):
            self.y[i] = self.gaussian(self.x[i], self.pose[0], self.sigma_movimento)
        self.ax.clear()
        self.ax.set_ylim([0, 4])
        self.ax.plot(self.x, self.y, color="b")
        plt.pause(0.1)

      if self.cmd > 0:
            self.controle = 1

      if len(leitura) == 0:
          pass
      else:
        if self.controle == 1:
            self.sigma_movimento = self.sigma_movimento + 0.002
        if (leitura[72] >= 1.50 and leitura[72] <= 1.60) and (leitura[108] >= 1.50 and leitura[108] <= 1.60):
            media_nova = (self.mapa[self.porta] * self.sigma_movimento + self.pose[0]*self.sigma_lidar) / (self.sigma_movimento+self.sigma_lidar)
            sigma_novo = 1 / (1/self.sigma_movimento + 1/self.sigma_lidar)

            self.pose[0] = media_nova # a nova posição x do robô

            for i in range(len(self.x)):
                self.y2[i] = self.gaussian(self.x[i], self.mapa[self.porta], self.sigma_lidar)
                self.ax.plot(self.x, self.y2, color="r")
                plt.pause(0.1)

            sigma_movimento = sigma_novo

            for i in range(len(self.x)):
                self.y3[i] = self.gaussian(self.x[i], media_nova, sigma_novo)
                self.ax.plot(self.x, self.y3, color="g")
                plt.pause(0.1)

            if self.porta == 0:
                self.porta = 1 # altera para a próxima porta 0 → 1 ; 1 → 2

            elif self.porta == 1:
                self.porta = 2

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
