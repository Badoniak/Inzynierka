#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np
from std_msgs.msg import Float32MultiArray

# Funkcja pomocnicza do trójkątów rozmytych
def triangle(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if a < x <= b: return (x - a) / (b - a)
    if b < x < c: return (c - x) / (c - b)
    return 0.0

# Konwersja Quaternion -> Yaw (kąt obrotu)
def get_yaw_from_odom(msg):
    orientation_q = msg.pose.pose.orientation
    x, y, z, w = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class FuzzyGoToGoal(Node):
    def __init__(self):
        super().__init__('fuzzy_goto_node')

        self.pos_x_sub =self.create_subscription(Float32MultiArray, '/target_topic', self.pos_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # --- CEL (Współrzędne w ROS: X=przód, Y=lewo) ---
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_target = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False


    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = get_yaw_from_odom(msg)
        self.has_odom = True

    def pos_callback(self, msg):

        if len(msg.data) >= 2:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.has_target = True
            self.get_logger().info(f"New target: X={self.target_x:.2f}, Y={self.target_y:.2f}")
        else:
            self.get_logger().error(f"Bad target message")
    # --- LOGIKA ROZMYTA ---

    def mu_distance(self, x):
        # Near: blisko celu (zwalniamy)
        N = triangle(x, -1.0, 0.0, 2.0)
        # Far: daleko (jedziemy szybko)
        F = triangle(x, 1.0, 5.0, 1000.0)
        return N, F
    
    def mu_angle(self, x):
        # error w zakresie [-pi, pi]. 
        # Ujemne = Cel po PRAWEJ. Dodatnie = Cel po LEWEJ.
        
        # Right: Pełna przynależność przy -1.5 rad (~-90st), znika przy 0
        R = triangle(x, -3.5, -1.5, 0.0)
        
        # Center: Pełna przy 0, znika przy +/- 0.5 rad
        C = triangle(x, -0.25, 0.0, 0.25)
        
        # Left: Pełna przy 1.5 rad (~90st)
        L = triangle(x, 0.0, 1.5, 3.5)
        
        return R, C, L
    
    RULES = {
        ("N", "R"):(0.35,-1.5),
        ("N", "L"):(0.35, 1.5),
        ("N", "C"):(0.35, 0.0),

        ("F", "R"):(1.5,-1.75),
        ("F", "L"):(1.5, 1.75),
        ("F", "C"):(1.5, 0.0),
    }

    def control_loop(self):
        if not self.has_odom: return

        if not self.has_target:
            self.get_logger().info(f"Waiting for instructions")
            return
        # 1. Oblicz błąd odległości i kąta
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_yaw
        
        # Normalizacja kąta do [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Jeśli jesteśmy super blisko, STOP
        if distance < 0.1:
            self.stop_robot()
            return

        # 2. Fuzyfikacja
        N_d, F_d = self.mu_distance(distance)
        R_a, C_a, L_a = self.mu_angle(angle_error)


        # 3. Wnioskowanie (Reguły)
        # Wyjścia: Linear (m/s), Angular (rad/s)
        memb_distance  = {'N': N_d, 'F': F_d}
        memb_angle = {'R': R_a, 'C': C_a, 'L': L_a}


        numerator_u1 = 0.0
        numerator_u2 = 0.0
        denominator = 0.0

        # 4. Defuzyfikacja (Średnia ważona)
        for (key_d, key_a), (out_u1, out_u2) in self.RULES.items():
            
            # Pobranie wartości przynależności dla danej reguły
            mu_d = memb_distance[key_d]
            mu_a = memb_angle[key_a]
            
            # Obliczenie stopnia odpalenia reguły (Beta) - Iloczyn (Product)
            beta = mu_d * mu_a

            # Jeśli reguła jest aktywna (beta > 0), dodajemy do sumy ważonej
            if beta > 0.0:
                numerator_u1 += beta * out_u1
                numerator_u2 += beta * out_u2
                denominator += beta

        msg = Twist()
        if denominator > 0:
            msg.linear.x = float(numerator_u1 / denominator)
            msg.angular.z = float(numerator_u2 / denominator)
        else:
            # Brak dopasowania reguł - obrót w miejscu szukając celu
            if angle_error > 0:
                msg.angular.z = 1.5
            else:
                msg.angular.z = -1.5

        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Dist: {distance:.2f}, AngErr: {angle_error:.2f}, Action: {msg.linear.x:.2f}, {msg.angular.z:.2f}")

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("CEL OSIĄGNIĘTY!")

def main(args=None):
    rclpy.init(args=args)
    node = FuzzyGoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
