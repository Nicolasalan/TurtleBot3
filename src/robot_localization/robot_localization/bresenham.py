import time
from typing import List
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
import math

from matplotlib import pyplot as plt
from typing import Tuple

class MappingBresenham(Node):

    def __init__(self):
        super().__init__('mapping')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.x = None
        self.y = None
        self.theta = None
        self.angles = []
        self.measurements = []
        self.map_measurements = np.ones((350, 300)) / 2

        self.xy_resolution = 0.02
        self.map_size = (350, 300)  # Tamanho do mapa (local)
        self.map = np.ones(self.map_size) / 2  # Mapa inicial
        self.fixed_obstacles = []
        self.obstacle_plot = None

    def listener_callback_laser(self, msg):
        # remova valores inf
        self.measurements = [m for m in msg.ranges if not np.isinf(m)]

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        num_measurements = len(self.measurements)
        # para cada angulo de medição, calcula o angulo absoluto
        self.angles = [angle_min + i * angle_increment for i in range(num_measurements)]


    def listener_callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta  = 2 * math.atan2(msg.pose.pose.orientation.y, msg.pose.pose.orientation.w)

    def navigation_start(self):
        self.get_logger().info("Inicialize seu código aqui!")

    def bresenham(self, start: Tuple[int], end: Tuple[int]) -> np.ndarray:
        """
        Implementação do algoritmo de desenho de linha de Bresenham
        -> wikipedia.org/wiki/Bresenham's_line_algorithm
        Algoritmo de Linha de Bresenham
        Produz um np.array do início e do fim
        >>> pontos = bresenham((4, 4), (6, 10))
        >>> imprimir(pontos)
        array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # configura condições iniciais
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx) # determina o quão íngreme é a linha
        if is_steep:  # girar linha
            # inverte os valores
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # troque os pontos inicial e final, se necessário, e armazene o estado de troca
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalcular diferenciais
        dy = y2 - y1  # recalcular diferenciais
        error = int(dx / 2.0)  # calcular o erro
        y_step = 1 if y1 < y2 else -1
        # iterar sobre a caixa delimitadora gerando pontos entre o início e o fim
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverter a lista se as coordenadas foram trocadas
            points.reverse()
        points = np.array(points)
        return points

    def calc_grid_map_config(self, ox, oy, xy_resolution):
        """
        Calcula o tamanho e as distâncias máximas de acordo com o
        centro de medição
        """

        EXTEND_AREA = 1.0

        min_x = round(min(ox) - EXTEND_AREA / 2.0)
        min_y = round(min(oy) - EXTEND_AREA / 2.0)
        max_x = round(max(ox) + EXTEND_AREA / 2.0)
        max_y = round(max(oy) + EXTEND_AREA / 2.0)
        xw = int(round((max_x - min_x) / xy_resolution))
        yw = int(round((max_y - min_y) / xy_resolution))
        return min_x, min_y, max_x, max_y, xw, yw

    def generate_ray_casting_grid_map(self, ox, oy, xy_resolution, bresen=True):
        min_x, min_y, max_x, max_y, x_w, y_w = self.calc_grid_map_config(
            ox, oy, xy_resolution)

        occupancy_map = np.ones((350, 300)) / 2
        center_x = int(
            round(-min_x / xy_resolution))
        center_y = int(
            round(-min_y / xy_resolution))

        for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))

            if 0 <= ix < occupancy_map.shape[0] and 0 <= iy < occupancy_map.shape[1]:
                laser_beams = self.bresenham((center_x, center_y), (ix, iy))
                for laser_beam in laser_beams:
                    if 0 <= laser_beam[0] < occupancy_map.shape[0] and 0 <= laser_beam[1] < occupancy_map.shape[1]:
                        occupancy_map[laser_beam[0]][laser_beam[1]] = 0.0
                occupancy_map[ix][iy] = 1.0
                if ix + 1 < occupancy_map.shape[0] and iy + 1 < occupancy_map.shape[1]:
                    occupancy_map[ix + 1][iy] = 1.0
                    occupancy_map[ix][iy + 1] = 1.0
                    occupancy_map[ix + 1][iy + 1] = 1.0

        return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution

    def update_map(self):
        xyreso = 0.02
        robot_pos = (self.x, self.y)
        ox = np.sin(self.angles) * self.measurements
        oy = np.cos(self.angles) * self.measurements

        pmap, minx, maxx, miny, maxy, xyreso = self.generate_ray_casting_grid_map(ox, oy, xyreso, True)

        self.map = pmap

        plt.imshow(self.map.T, cmap='gray', origin='lower')
        plt.pause(0.001)

    def navigation_update(self):
        if len(self.measurements) == 0:
            pass
        else:
            self.update_map()


def main(args=None):
    rclpy.init(args=args)

    navigator = MappingBresenham()

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
