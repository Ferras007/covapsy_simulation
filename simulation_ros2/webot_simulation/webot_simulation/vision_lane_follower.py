#!/usr/bin/env python3
# Auteur: Ferras NAIMI (M2 ISI)
#
# Navigation autonome basée sur la fusion caméra depth / LiDAR.
# La caméra est utilisée pour l’anticipation locale,
# le LiDAR sert principalement à la sécurité et à la robustesse.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge

import numpy as np
import math


class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('autonomous_navigation')

        # ================== PARAMÈTRES ==================
        # Depth (m)
        self.depth_threshold = 1.2        # obstacle "devant" caméra
        self.k_side = 0.15                # anticipation latérale (virage tôt)

        # Steering
        self.steering_gain = 0.35         # braquage fort quand obstacle devant
        self.steer_smooth = 0.7           # lissage (0..1), plus grand = plus doux
        self.max_steer = 0.45             # limite physique (rad)

        # LiDAR sécurité
        self.lidar_stop_dist = 0.35       # STOP si obstacle très proche
        self.lidar_stop_count_req = 3     # nb mesures consécutives pour STOP

        # Vitesse
        self.max_speed = 0.30             # vitesse nominale
        self.turn_slow_factor = 0.7       # ralentir en virage (sans STOP)

        # ================== ÉTAT ==================
        self.depth_left = None
        self.depth_center = None
        self.depth_right = None

        self.lidar_front = float('inf')
        self.lidar_stop_counter = 0

        self.prev_steering = 0.0

        # ================== ROS ==================
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/TT02_jaune/rb5_depth/image',
            self.depth_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/TT02_jaune/RpLidarA2',
            self.lidar_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            AckermannDrive,
            '/cmd_ackermann',
            10
        )

        self.get_logger().info("✅ Autonomous navigation (FINAL) started")

    # ============================================================
    # LIDAR → SÉCURITÉ (front_min)
    # ============================================================
    def lidar_callback(self, scan: LaserScan):

        def angle_to_index(angle):
            # Gère angle_increment négatif aussi
            i = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(i, len(scan.ranges) - 1))

        # secteur devant : -10° → +10°
        a0 = math.radians(-10)
        a1 = math.radians(+10)

        i0 = angle_to_index(a0)
        i1 = angle_to_index(a1)
        if i0 > i1:
            i0, i1 = i1, i0

        vals = []
        for r in scan.ranges[i0:i1+1]:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                vals.append(r)

        self.lidar_front = min(vals) if vals else scan.range_max

    # ============================================================
    # DEPTH → PERCEPTION (gauche / centre / droite)
    # ============================================================
    def depth_callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        h, w = depth.shape
        third = w // 3

        left   = depth[:, :third]
        center = depth[:, third:2*third]
        right  = depth[:, 2*third:]

        def dist(zone):
            z = zone[np.isfinite(zone)]
            return float(np.median(z)) if z.size else float('inf')

        self.depth_left   = dist(left)
        self.depth_center = dist(center)
        self.depth_right  = dist(right)

        self.navigate()

    # ============================================================
    # FUSION + COMMANDE
    # ============================================================
    def navigate(self):

        # Fail-safe
        if self.depth_center is None:
            return

        # ---------------- STEERING (CAMÉRA) ----------------
        # Anticipation latérale (tourne tôt)
        side_diff = self.depth_left - self.depth_right
        steering = -self.k_side * side_diff

        # Décision forte si obstacle vraiment devant
        if self.depth_center < self.depth_threshold:
            if self.depth_left > self.depth_right:
                steering = -self.steering_gain
            else:
                steering = +self.steering_gain

        # Saturation
        steering = max(-self.max_steer, min(self.max_steer, steering))

        # Lissage
        steering = self.steer_smooth * self.prev_steering + (1.0 - self.steer_smooth) * steering
        self.prev_steering = steering

        # ---------------- SPEED (LiDAR robuste) ----------------
        if self.lidar_front < self.lidar_stop_dist:
            self.lidar_stop_counter += 1
        else:
            self.lidar_stop_counter = 0

        if self.lidar_stop_counter >= self.lidar_stop_count_req:
            speed = 0.0
        else:
            speed = self.max_speed

        # Ne jamais STOP brutalement en virage (ralentir seulement)
        if abs(steering) > 0.1 and speed > 0.0:
            speed *= self.turn_slow_factor

        # ---------------- COMMANDE ----------------
        cmd = AckermannDrive()
        cmd.speed = speed
        cmd.steering_angle = steering
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"DEPTH L={self.depth_left:.2f} C={self.depth_center:.2f} R={self.depth_right:.2f} | "
            f"LIDAR front={self.lidar_front:.2f} | "
            f"cmd v={speed:.2f} steer={steering:.2f}"
        )


def main():
    rclpy.init()
    node = AutonomousNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
