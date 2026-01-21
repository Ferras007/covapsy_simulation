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

        # ---------------- PARAMÈTRES CAMÉRA ----------------
        # Distance seuil en dessous de laquelle un obstacle est considéré "devant"
        self.depth_threshold = 1.2

        # Gain utilisé pour anticiper les variations gauche/droite
        self.k_side = 0.18

        # ---------------- PARAMÈTRES DE DIRECTION ----------------
        # Braquage utilisé quand une décision franche est nécessaire
        self.steering_gain = 0.40

        # Limite physique du braquage (liée au modèle Ackermann)
        self.max_steer = 0.45

        # Lissage du braquage pour éviter les oscillations
        self.steer_smooth = 0.7

        # Décroissance du biais de direction pour éviter les à-coups
        self.bias_decay = 0.96

        # ---------------- PARAMÈTRES LiDAR (SÉCURITÉ) ----------------
        # Distance minimale devant le véhicule avant arrêt
        self.lidar_stop_dist = 0.35

        # Nombre de mesures consécutives requises avant un arrêt complet
        self.lidar_stop_count_req = 3

        # ---------------- PARAMÈTRES DE VITESSE ----------------
        self.max_speed = 0.30
        self.turn_slow_factor = 0.7

        # ---------------- MODE VIRAGE SERRÉ ----------------
        # Mode spécifique activé lorsque la caméra détecte une fermeture rapide
        self.in_sharp_turn = False
        self.sharp_turn_dir = 0.0

        # Seuils d’entrée/sortie du mode virage serré (hystérésis)
        self.sharp_turn_enter_dist = 0.7
        self.sharp_turn_exit_dist = 1.6

        # ---------------- ÉTATS CAPTEURS ----------------
        self.depth_left = None
        self.depth_center = None
        self.depth_right = None

        self.lidar_front = float('inf')
        self.lidar_stop_counter = 0

        # Mémoire du braquage précédent pour le lissage
        self.prev_steering = 0.0

        # Biais temporaire conservé lorsque la caméra hésite
        self.turn_bias = 0.0

        # ---------------- ROS ----------------
        self.bridge = CvBridge()

        # Abonnement caméra depth
        self.create_subscription(
            Image,
            '/TT02_jaune/rb5_depth/image',
            self.depth_callback,
            10
        )

        # Abonnement LiDAR
        self.create_subscription(
            LaserScan,
            '/TT02_jaune/RpLidarA2',
            self.lidar_callback,
            10
        )

        # Publication des commandes Ackermann
        self.cmd_pub = self.create_publisher(
            AckermannDrive,
            '/cmd_ackermann',
            10
        )

        self.get_logger().info("vision_lane_follower — navigation active")

    # =====================================================
    # CALLBACK LiDAR
    # Extraction de la distance minimale dans un secteur frontal
    # =====================================================
    def lidar_callback(self, scan: LaserScan):

        def angle_to_index(angle):
            i = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(i, len(scan.ranges) - 1))

        # Secteur frontal ±10°
        a0 = math.radians(-10)
        a1 = math.radians(+10)

        i0 = angle_to_index(a0)
        i1 = angle_to_index(a1)
        if i0 > i1:
            i0, i1 = i1, i0

        vals = []
        for r in scan.ranges[i0:i1 + 1]:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                vals.append(r)

        # Distance frontale minimale utilisée pour la sécurité
        self.lidar_front = min(vals) if vals else scan.range_max

    # =====================================================
    # CALLBACK CAMÉRA DEPTH
    # Découpage de l’image en trois zones horizontales
    # =====================================================
    def depth_callback(self, msg: Image):

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth.shape
        third = w // 3

        left   = depth[:, :third]
        center = depth[:, third:2 * third]
        right  = depth[:, 2 * third:]

        # Utilisation de la médiane pour réduire l’impact du bruit
        def dist(zone):
            z = zone[np.isfinite(zone)]
            return float(np.median(z)) if z.size else float('inf')

        self.depth_left   = dist(left)
        self.depth_center = dist(center)
        self.depth_right  = dist(right)

        # Déclenchement de la logique de navigation
        self.navigate()

    # =====================================================
    # LOGIQUE DE NAVIGATION
    # =====================================================
    def navigate(self):

        # Sécurité : attendre d’avoir des données valides
        if self.depth_center is None:
            return

        # ---------- DÉTECTION D’UN VIRAGE SERRÉ ----------
        if self.depth_center < self.sharp_turn_enter_dist and not self.in_sharp_turn:
            self.in_sharp_turn = True

            # La direction du virage est choisie selon l’espace disponible
            self.sharp_turn_dir = -1.0 if self.depth_left > self.depth_right else +1.0

        if self.in_sharp_turn and self.depth_center > self.sharp_turn_exit_dist:
            self.in_sharp_turn = False
            self.sharp_turn_dir = 0.0

        # ---------- CALCUL DU BRAQUAGE ----------
        if self.in_sharp_turn:
            # En virage serré, on impose un braquage constant et franc
            steering = self.sharp_turn_dir * self.max_steer
        else:
            # Anticipation latérale basée sur la différence gauche/droite
            side_diff = self.depth_left - self.depth_right
            steering = -self.k_side * side_diff

            # Obstacle détecté devant : décision plus agressive
            if self.depth_center < self.depth_threshold:
                if self.depth_left > self.depth_right:
                    steering = -self.steering_gain
                else:
                    steering = +self.steering_gain
                self.turn_bias = steering

            # En cas d’ambiguïté, on conserve le dernier biais connu
            if abs(steering) < 0.05:
                steering = self.turn_bias

            # Atténuation progressive du biais
            self.turn_bias *= self.bias_decay

        # Saturation du braquage
        steering = max(-self.max_steer, min(self.max_steer, steering))

        # Lissage temporel
        steering = (
            self.steer_smooth * self.prev_steering
            + (1.0 - self.steer_smooth) * steering
        )
        self.prev_steering = steering

        # ---------- GESTION DE LA VITESSE ----------
        if self.in_sharp_turn:
            speed = self.max_speed * 0.4
        else:
            if self.lidar_front < self.lidar_stop_dist:
                self.lidar_stop_counter += 1
            else:
                self.lidar_stop_counter = 0

            if self.lidar_stop_counter >= self.lidar_stop_count_req:
                speed = 0.0
            else:
                speed = self.max_speed

            # Ralentissement en virage
            if abs(steering) > 0.1 and speed > 0.0:
                speed *= self.turn_slow_factor

        # ---------- COMMANDE FINALE ----------
        cmd = AckermannDrive()
        cmd.speed = speed
        cmd.steering_angle = steering
        self.cmd_pub.publish(cmd)

    def main():
        rclpy.init()
        node = AutonomousNavigation()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
