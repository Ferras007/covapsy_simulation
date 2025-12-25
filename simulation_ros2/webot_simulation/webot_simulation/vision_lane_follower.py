#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# If you use Ackermann:
from ackermann_msgs.msg import AckermannDriveStamped


class VisionLaneFollower(Node):
    def __init__(self):
        super().__init__('vision_lane_follower')

        # -------- Parameters --------
        self.declare_parameter('image_topic', '/TT02_jaune/rb5_rgb/image_color')
        self.declare_parameter('cmd_topic', '/cmd_ackermann')

        # ROI (bottom part of image)
        self.declare_parameter('roi_y_start_ratio', 0.55)   # start of ROI (0..1)
        self.declare_parameter('roi_y_end_ratio', 1.00)     # end of ROI (0..1)

        # HSV thresholds (tune if needed)
        # You can follow ONE color (e.g. green) or combine red+green.
        self.declare_parameter('use_two_lines', True)  # if you have red+green corridor
        # Green
        self.declare_parameter('green_hsv_low',  [35, 60, 40])
        self.declare_parameter('green_hsv_high', [85, 255, 255])
        # Red (two ranges in HSV)
        self.declare_parameter('red1_hsv_low',   [0, 70, 40])
        self.declare_parameter('red1_hsv_high',  [10, 255, 255])
        self.declare_parameter('red2_hsv_low',   [170, 70, 40])
        self.declare_parameter('red2_hsv_high',  [180, 255, 255])

        # Control gains
        self.declare_parameter('kp', 0.9)
        self.declare_parameter('kd', 0.10)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('max_steer', 0.35)     # rad
        self.declare_parameter('base_speed', 2.5)     # m/s (sim)
        self.declare_parameter('min_speed',  0.8)
        self.declare_parameter('slowdown_gain', 2.0)  # slow more when turning

        # Debug
        self.declare_parameter('debug_view', True)

        # -------- Internal state --------
        self.bridge = CvBridge()
        self.prev_err = 0.0
        self.err_i = 0.0
        self.prev_time = self.get_clock().now()

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        self.get_logger().info(f"Listening image: {self.image_topic}")
        self.get_logger().info(f"Publishing cmd:  {self.cmd_topic}")

    def _mask_color(self, hsv, low, high):
        low = np.array(low, dtype=np.uint8)
        high = np.array(high, dtype=np.uint8)
        return cv2.inRange(hsv, low, high)

    def _largest_blob_center(self, mask):
        # Morphology to clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0

        c = max(contours, key=cv2.contourArea)
        area = int(cv2.contourArea(c))
        if area < 200:  # too small
            return None, area

        M = cv2.moments(c)
        if M["m00"] < 1e-5:
            return None, area
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy), area

    def on_image(self, msg: Image):
        # Time delta
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 1e-3
        self.prev_time = now

        # Convert
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w = bgr.shape[:2]

        # ROI
        y0 = int(h * self.get_parameter('roi_y_start_ratio').value)
        y1 = int(h * self.get_parameter('roi_y_end_ratio').value)
        roi = bgr[y0:y1, :, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        use_two = bool(self.get_parameter('use_two_lines').value)

        # Masks
        g_low = self.get_parameter('green_hsv_low').value
        g_high = self.get_parameter('green_hsv_high').value
        green_mask = self._mask_color(hsv, g_low, g_high)

        r1_low = self.get_parameter('red1_hsv_low').value
        r1_high = self.get_parameter('red1_hsv_high').value
        r2_low = self.get_parameter('red2_hsv_low').value
        r2_high = self.get_parameter('red2_hsv_high').value
        red_mask = self._mask_color(hsv, r1_low, r1_high) | self._mask_color(hsv, r2_low, r2_high)

        # Find centers
        green_center, green_area = self._largest_blob_center(green_mask)
        red_center, red_area = self._largest_blob_center(red_mask)

        # Compute desired center x (lane center)
        target_x = w // 2
        lane_x = None

        if use_two:
            # corridor center = midpoint between red & green when both visible
            if green_center and red_center:
                lane_x = int((green_center[0] + red_center[0]) / 2)
            elif green_center:
                # only green visible -> keep a safe offset to stay between
                lane_x = int(green_center[0] - 0.20 * w)
            elif red_center:
                lane_x = int(red_center[0] + 0.20 * w)
        else:
            # follow green only
            if green_center:
                lane_x = green_center[0]
            elif red_center:
                lane_x = red_center[0]

        # If nothing detected -> slow down + keep previous steering
        if lane_x is None:
            steer = float(np.clip(self.prev_err * -1.0, -self.get_parameter('max_steer').value, self.get_parameter('max_steer').value))
            speed = float(self.get_parameter('min_speed').value)
            self._publish_cmd(speed, steer)
            self._debug_show(roi, green_mask, red_mask, None, target_x, lane_x, speed, steer)
            return

        # Error: positive if lane is to the right
        err = (lane_x - target_x) / float(w)  # normalized [-0.5..0.5]
        # PID
        kp = float(self.get_parameter('kp').value)
        kd = float(self.get_parameter('kd').value)
        ki = float(self.get_parameter('ki').value)

        derr = (err - self.prev_err) / dt
        self.err_i += err * dt
        self.prev_err = err

        steer = -(kp * err + kd * derr + ki * self.err_i)  # negative -> steer towards lane center
        max_steer = float(self.get_parameter('max_steer').value)
        steer = float(np.clip(steer, -max_steer, max_steer))

        # Speed: slow down when steering a lot
        base_speed = float(self.get_parameter('base_speed').value)
        min_speed = float(self.get_parameter('min_speed').value)
        slowdown_gain = float(self.get_parameter('slowdown_gain').value)
        speed = base_speed * (1.0 - slowdown_gain * min(abs(steer) / max_steer, 1.0))
        speed = float(np.clip(speed, min_speed, base_speed))

        self._publish_cmd(speed, steer)
        self._debug_show(roi, green_mask, red_mask, (lane_x, int((y1-y0)*0.6)), target_x, lane_x, speed, steer)

    def _publish_cmd(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)

    def _debug_show(self, roi, green_mask, red_mask, lane_pt, target_x, lane_x, speed, steer):
        if not bool(self.get_parameter('debug_view').value):
            return

        vis = roi.copy()
        h, w = vis.shape[:2]

        # Draw target center
        cv2.line(vis, (target_x, 0), (target_x, h-1), (255, 255, 255), 2)

        # Draw lane point
        if lane_x is not None:
            cv2.circle(vis, (lane_x, int(h*0.7)), 8, (0, 255, 255), -1)

        txt = f"speed={speed:.2f} steer={steer:.3f}"
        cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        # Show masks small
        g = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
        r = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        g = cv2.resize(g, (w//3, h//3))
        r = cv2.resize(r, (w//3, h//3))
        vis[0:g.shape[0], 0:g.shape[1]] = g
        vis[0:r.shape[0], g.shape[1]:g.shape[1]+r.shape[1]] = r

        cv2.imshow("lane_follower_debug", vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
