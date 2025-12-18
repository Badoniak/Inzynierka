#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import time


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('linear_speed', 1.2)
        self.declare_parameter('angular_speed', 1.25)
        self.declare_parameter('safe_distance', 1.7)
        self.declare_parameter('danger_distance', 1.1)
        self.declare_parameter('scan_topic', '/LaserScan')
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel')

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.latest_scan = None
        self.mode = "forward"  # forward / rotate
        self.turn_direction = 0
        self.last_turn_time = 0.0

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_sector_distance_wrap(self, ranges, angles, min_angle, max_angle):
        if min_angle < max_angle:
            mask = (angles >= min_angle) & (angles <= max_angle)
        else:
        # zakres przez 0 rad → np. 350°–10°
            mask = (angles >= min_angle) | (angles <= max_angle)
        valid = ranges[mask]
        finite = valid[np.isfinite(valid)]
        if len(finite) == 0:
            return float('inf')
        return np.min(finite)

    def analyze_scan(self):
        if self.latest_scan is None:
            return None

        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
	
        valid_mask = np.isfinite(ranges) & (ranges > 0.01)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return False, 0
	
	
        right = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(11), math.radians(55)) - 0.58
        left = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(305), math.radians(349)) - 0.58
        front = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(350), math.radians(10)) - 0.5

        return front, left, right

    def control_loop(self):
        scan = self.analyze_scan()
        if scan is None:
            return

        front, left, right = scan
        cmd = Twist()
        now = time.time()
        diff = 0.0
        # Różnica odległości między stronami
        if np.isfinite(left) and np.isfinite(right):
            diff = left - right
        elif np.isfinite(left):
            diff = 0.8  # widzi tylko lewą ścianę → skręć lekko w prawo
        elif np.isfinite(right):
            diff = -0.8  # widzi tylko prawą ścianę → skręć lekko w lewo
        else:
            diff = 0.0

        # 1️⃣ Wybór trybu
        if front < self.danger_distance:
        # krytycznie blisko - obrót w miejscu
            if self.mode != "rotate":  # kierunek ustalany tylko raz na wejściu w tryb rotate
                self.turn_direction = 1 if left > right else -1
                self.get_logger().info(f"🚨 Blisko przeszkoda, obracam w kierunku: {'LEFT' if self.turn_direction>0 else 'RIGHT'}")
            self.mode = "rotate"

        elif front < self.safe_distance:
            # Przeszkoda w zasięgu — jazda korygująca między ścianami
            self.mode = "avoid"

        else: 
            if abs(diff) < 0.15:
                self.mode = "forward"
            else:
            	self.mode = "avoid"	
           # 2️⃣ Zachowanie dla każdego trybu
        if self.mode == "forward":
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        elif self.mode == "rotate":
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * self.turn_direction
            if front > self.safe_distance * 1.2:
                self.mode = "avoid"

        elif self.mode == "avoid":

            # Normalizacja błędu do zakresu [-1, 1]
            diff = max(min(diff, 1.0), -1.0)

            # Korekcja toru proporcjonalna do różnicy
            correction = 0.08 + (diff * self.angular_speed * 1.15)  # współczynnik czułości

            # Prędkość do przodu maleje, jeśli przeszkoda z przodu blisko
            speed_factor = max(0.2, min(front / self.safe_distance, 1.0))
            cmd.linear.x = self.linear_speed * speed_factor
            cmd.angular.z = correction  # ujemny znak: skręt przeciwny do większej odległości

        # 3️⃣ Publikacja i logowanie
        self.vel_pub.publish(cmd)

        if not hasattr(self, "_counter"):
            self._counter = 0
        self._counter += 1
        if self._counter % 5 == 0:
            self.get_logger().info(
                f"Mode={self.mode}, front={front:.2f}, L={left:.2f}, R={right:.2f}, diff={left - right:.2f}, cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})"
            )



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

