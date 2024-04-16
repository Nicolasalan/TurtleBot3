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

class Mapping(Node):

    def __init__(self):
        super().__init__('mapping')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.x = None
        self.y = None
        self.theta = None
        self.range_max = None
        self.angles = []
        self.measurements = []
        self.map_measurements = np.ones((350, 300)) / 2

    def listener_callback_laser(self, msg):
        self.measurements = [msg.range_max if np.isinf(m) else m for m in msg.ranges]

        # Obter o ângulo inicial e o incremento angular
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Calcular os ângulos correspondentes a cada medida de distância
        num_measurements = len(self.measurements)
        self.angles = [angle_min + i * angle_increment for i in range(num_measurements)]

        self.range_max = msg.range_max

    def listener_callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta  = 2 * math.atan2(msg.pose.pose.orientation.y, msg.pose.pose.orientation.w)

    def navigation_start(self):
        self.get_logger().info("Inicialize seu código aqui!")

    def bresenham(self, start, end):
        """
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points)
        return points

    def calc_grid_map_config(self, ox, oy, xy_resolution):
        """
        Calculates the size, and the maximum distances according to the the
        measurement center
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

        # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
        occupancy_map = np.ones((350, 300)) / 2
        center_x = int(
            round(-min_x / xy_resolution))  # center x coordinate of the grid map
        center_y = int(
            round(-min_y / xy_resolution))  # center y coordinate of the grid map

        for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))

            # Verifique se as coordenadas estão dentro dos limites da matriz atual
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
            # else:
            #     # Expanda a matriz para incluir a nova posição do robô
            #     new_width = max(occupancy_map.shape[0], ix + 10)  # aumente em 10 células
            #     new_height = max(occupancy_map.shape[1], iy + 10)  # aumente em 10 células
            #     expanded_map = np.ones((new_width, new_height)) / 2
            #     # Copie os valores da matriz original para a nova matriz
            #     expanded_map[:occupancy_map.shape[0], :occupancy_map.shape[1]] = occupancy_map
            #     occupancy_map = expanded_map

        return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution

    def navigation_update(self):
        xyreso = 0.02  # x-y grid resolution
        if len(self.measurements) == 0:
            pass
        else:
            odometry = [self.x, self.y, self.theta]
            #plt.clf()  # Clear the current figure.

            xyreso = 0.02
            ox = np.sin(self.angles) * self.measurements
            oy = np.cos(self.angles) * self.measurements
            pmap, minx, maxx, miny, maxy, xyreso = self.generate_ray_casting_grid_map(ox, oy, xyreso, True)

            self.map_measurements += pmap

            # numpy.ndarray
            # (350, 300)

            # Adiciona pmap a self.map_measurements
            #if not hasattr(self, 'map_measurements'):
            #    self.map_measurements = pmap
            #else:
            #    self.map_measurements += pmap

            plt.imshow(pmap.T, cmap='gray', origin='lower')
            plt.pause(0.001)  # Pause to update the figure.


def main(args=None):
    rclpy.init(args=args)

    navigator = Mapping()

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
