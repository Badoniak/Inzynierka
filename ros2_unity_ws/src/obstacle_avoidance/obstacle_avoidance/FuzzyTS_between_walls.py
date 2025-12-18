#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import time


def triangle(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if a < x < b:
        return (x - a) / (b - a)
    if b <= x < c:
        return (c - x) / (c - b)
    return 0.0

class TSObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('safe_distance', 2.2)
        self.declare_parameter('danger_distance', 1.5)
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
	
	
        right = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(25), math.radians(110)) - 0.58
        left = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(270), math.radians(345)) - 0.58
        front = self.get_sector_distance_wrap(valid_ranges, valid_angles,math.radians(350), math.radians(10)) - 0.5

        return front, left, right

    def mu_front(self, x):
        # S (Small): Hamuj awaryjnie poniżej 1.0m. Pełna aktywacja przy 0.
        S = triangle(x, -1.0, 0.0, 1.3)
        # M (Medium): Ostrzeżenie między 0.8m a 3.0m
        M = triangle(x, 0.8, 2.0, 3.5)
        # L (Large): Bezpiecznie powyżej 2.5m
        L = triangle(x, 2.5, 5.0, 20.0)
        return S, M, L

    def mu_side(self, x):
        # C (Close): BARDZO BLISKO. Reakcja natychmiastowa poniżej 0.6m.
        # Powyżej 0.75m to już NIE jest Close.
        C = triangle(x, -1.0, 0.0, 0.75)
        
        # M (Medium): Korytarz / jazda przy ścianie (0.5m - 1.8m).
        # Tu robot ma się delikatnie centrować, a nie panikować.
        M = triangle(x, 0.4, 1.0, 2.0)
        
        # F (Far): Wolna przestrzeń powyżej 1.5m
        F = triangle(x, 1.5, 3.0, 20.0)
        return C, M, F
    # ---------------- RULE TABLE ----------------
    # (x1:front, x2:left, x3:right) -> u1:x_lin(m/s), u2:z_ang(rad/s))
    # u1: 0/0.5/1.0 m/s  ;  u2: -1.5/0/+1.5 rad/s
    RULES = {
        # --- SYTUACJA KRYTYCZNA Z PRZODU (S) ---
        # Priorytet: Obrót w miejscu, prędkość liniowa minimalna
        ("S","C","C"):(0.0, -1.6), # Ślepy zaułek -> obrót (można dać losowo lub w preferowaną stronę)
        ("S","C","M"):(0.4, -1.4), # Uciekaj w prawo (tam luźniej)
        ("S","C","F"):(0.4, -1.7), # Uciekaj w prawo
        ("S","M","C"):(0.4, +1.4), # Uciekaj w lewo (tam luźniej)
        ("S","M","M"):(0.4, -1.7), # Przeszkoda z przodu, boki średnio -> duży skręt
        ("S","M","F"):(0.4, -1.7), # Skręt w wolne
        ("S","F","C"):(0.4, +1.7), # Skręt w wolne
        ("S","F","M"):(0.4, +1.7),
        ("S","F","F"):(0.4, +1.7), # Ściana tylko z przodu

        # --- BLISKO Z BOKU (C) - ALE PRZÓD WOLNY (M/L) ---
        # Priorytet: Zwolnij i zdecydowanie (ale nie panicznie) odbij
        ("M","C","C"):(0.6, -1.1), # Wąsko -> wolno i korekta
        ("M","C","M"):(0.7, -1.3), # Blisko z lewej -> w prawo
        ("M","C","F"):(0.8, -1.5), # Blisko z lewej -> w prawo (szybciej bo F wolne)
        ("M","M","C"):(0.7, +1.3), # Blisko z prawej -> w lewo
        ("M","F","C"):(0.8, +1.5), # Blisko z prawej -> w lewo (szybciej bo F wolne)
        # --- JAZDA KORYTARZEM / ŚREDNIO Z BOKÓW (M) ---
        # Priorytet: Płynność (małe skręty, duża prędkość) - TU BYŁ PROBLEM OSCYLACJI
        ("M","M","M"):(1.3, -0.5), # Jedź prosto
        ("M","M","F"):(1.3, -0.8), # Lekka korekta w prawo
        ("M","F","M"):(1.3, +0.8), # Lekka korekta w lewo
        ("M","F","F"):(1.5, -0.3),

        # Analogicznie dla L (Large Front) - tu możemy jechać szybciej
        ("L","C","C"):(0.9, -0.4), # Jedź środkiem
        ("L","C","M"):(0.9, -0.4), # Korekta w prawo
        ("L","C","F"):(1.0, -0.5), # Korekta w prawo
        ("L","M","C"):(0.9, +0.4), # Korekta w lewo
        ("L","F","C"):(1.0, +0.6), # Korekta w lewo

        
        ("L","M","M"):(1.4,  0.0), # Szybko prosto
        ("L","M","F"):(1.4, -0.5), # Bardzo delikatna korekta (anty-oscylacja)
        ("L","F","M"):(1.4, +0.5), # Bardzo delikatna korekta (anty-oscylacja)

        # --- OTWARTA PRZESTRZEŃ (F) ---
        ("L","F","F"):(1.5, 0.0), # Maksymalna prędkość
    }

    def control_loop(self):
        scan = self.analyze_scan()
        if scan is None:
            return

        front, left, right = scan
        # Logowanie dla celów debugowania
        self.get_logger().info(f"Dist: F={front:.2f}, L={left:.2f}, R={right:.2f}")

        # 2. Fuzyfikacja (Obliczenie przynależności do zbiorów)
        S_f, M_f, L_f = self.mu_front(front)
        C_l, M_l, F_l = self.mu_side(left)
        C_r, M_r, F_r = self.mu_side(right)

        # Słowniki pomocnicze do łatwego pobierania wartości na podstawie kluczy reguł
        memb_front = {'S': S_f, 'M': M_f, 'L': L_f}
        memb_left  = {'C': C_l, 'M': M_l, 'F': F_l}
        memb_right = {'C': C_r, 'M': M_r, 'F': F_r}

        numerator_u1 = 0.0  # Licznik dla prędkości liniowej
        numerator_u2 = 0.0  # Licznik dla prędkości kątowej
        denominator = 0.0   # Mianownik (suma wag beta)

        # 3. Wnioskowanie (Iteracja po regułach)
        for (key_f, key_l, key_r), (out_u1, out_u2) in self.RULES.items():
            
            # Pobranie wartości przynależności dla danej reguły
            mu_f = memb_front[key_f]
            mu_l = memb_left[key_l]
            mu_r = memb_right[key_r]

            # Obliczenie stopnia odpalenia reguły (Beta) - Iloczyn (Product)
            beta = mu_f * mu_l * mu_r

            # Jeśli reguła jest aktywna (beta > 0), dodajemy do sumy ważonej
            if beta > 0.0:
                numerator_u1 += beta * out_u1
                numerator_u2 += beta * out_u2
                denominator += beta

        # 4. Defuzyfikacja i wysłanie komendy
        msg = Twist()

        if denominator > 0.0:
            # Obliczenie średniej ważonej (Center of Sums / Height Method)
            final_u1 = numerator_u1 / denominator
            final_u2 = numerator_u2 / denominator
            
            msg.linear.x = float(final_u1)
            msg.angular.z = float(final_u2)
            
            self.get_logger().info(f"Action: LIN={final_u1:.2f}, ANG={final_u2:.2f}")
        else:
            # Sytuacja awaryjna: żadna reguła nie pasuje (np. jesteśmy bardzo daleko, poza zakresem funkcji L/F)
            # W tym przypadku bezpiecznie jest się zatrzymać lub jechać powoli
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().warn("Brak dopasowania reguł - STOP")

        self.vel_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = TSObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

