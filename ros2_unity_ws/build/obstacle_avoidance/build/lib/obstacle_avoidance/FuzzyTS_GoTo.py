#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math


# ------------------------
# Quaternion → yaw
# ------------------------
def get_yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# =====================================================
#        DECISION-BASED GO-TO-GOAL CONTROLLER
# =====================================================
class DecisionGoToGoal(Node):

    def __init__(self):
        super().__init__('decision_goto_goal')

        # --- SUB / PUB ---
        self.create_subscription(
            Float32MultiArray,
            '/target_topic',
            self.target_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        # --- STATE ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        self.target_x = 0.0
        self.target_y = 0.0
        self.has_target = False

        # --- PARAMETRY REGULATORA ---
        self.goal_tolerance = 0.15     # [m]

        self.fast_speed = 1.6          # >3m
        self.medium_speed = 1.0        # 2–3m
        self.slow_speed = 0.4          # <2m

        self.max_angular = 1.8         # [rad/s]

        self.get_logger().info("Decision-based GO-TO-GOAL controller started")

    # ------------------------
    # Callbacks
    # ------------------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = get_yaw_from_odom(msg)
        self.has_odom = True

    def target_callback(self, msg):
        if len(msg.data) >= 2:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.has_target = True
            self.get_logger().info(
                f"New target received: X={self.target_x:.2f}, Y={self.target_y:.2f}"
            )

    # ------------------------
    # Main control
    # ------------------------
    def control_loop(self):

        if not self.has_odom or not self.has_target:
            return

        # --- Obliczenia geometryczne ---
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_yaw
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        cmd = Twist()

        # =========================
        #  CEL OSIĄGNIĘTY
        # =========================
        if distance < self.goal_tolerance:
            self.stop_robot()
            return

        # =========================
        #  STREFA 1: DALEKO > 3 m
        # =========================
        if distance > 3.0:
            cmd.linear.x = self.fast_speed
            cmd.angular.z = angle_error
            mode = "FAST_CRUISE"

        # =========================
        #  STREFA 2: 2–3 m
        # =========================
        elif distance > 2.0:
            if abs(angle_error) > 0.5:
                cmd.linear.x = 0.6
                cmd.angular.z = 1.2 * angle_error
                mode = "ALIGN_APPROACH"
            else:
                cmd.linear.x = self.medium_speed
                cmd.angular.z = 0.8 * angle_error
                mode = "CONTROLLED_APPROACH"

        # =========================
        #  STREFA 3: < 2 m
        # =========================
        else:
            if abs(angle_error) > 0.3:
                cmd.linear.x = 0.2
                cmd.angular.z = 1.5 * angle_error
                mode = "FINAL_ALIGN"
            else:
                cmd.linear.x = self.slow_speed
                cmd.angular.z = 0.0
                mode = "FINAL_FORWARD"

        # --- Ograniczenie prędkości kątowej ---
        cmd.angular.z = max(
            -self.max_angular,
            min(self.max_angular, cmd.angular.z)
        )

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"[{mode}] dist={distance:.2f}  ang_err={angle_error:.2f}  "
            f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})"
        )

    # ------------------------
    # STOP
    # ------------------------
    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("TARGET REACHED")


# =====================================================
#                    MAIN
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = DecisionGoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

