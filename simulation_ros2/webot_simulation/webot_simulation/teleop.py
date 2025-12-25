import threading
import click

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

class KeyboardAckermannController(Node):
    def __init__(self):
        super().__init__('teleop_ackermann_node')

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.get_logger().info("Ackermann teleop node started. Use arrows to drive.")

        self.ackermann_pub = self.create_publisher(AckermannDrive, '/cmd_ackermann', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.speed = 0.0
        self.steering_angle = 0.0
        self.key_mapping = {
            '\x1b[A': 'UP',     # Arrow Up
            '\x1b[B': 'DOWN',   # Arrow Down
            '\x1b[C': 'RIGHT',  # Arrow Right
            '\x1b[D': 'LEFT',   # Arrow Left
            's': 'BRAKE',
            'q': 'QUIT',
            'n': 'NEUTRAL'
        }

    def timer_callback(self):
        msg = AckermannDrive()
        msg.speed = self.speed
        msg.steering_angle = self.steering_angle
        self.ackermann_pub.publish(msg)
        if self.debug:
            self.get_logger().info(f"[DEBUG] Publishing: speed={self.speed:.2f}, steering={self.steering_angle:.2f}")

    def perform_action(self):
        key = click.getchar()
        action = self.key_mapping.get(key)

        if action == 'UP':
            self.speed += 0.1
        elif action == 'DOWN':
            self.speed -= 0.1
        elif action == 'LEFT':
            self.steering_angle = max(self.steering_angle - 0.1, -0.35)
        elif action == 'RIGHT':
            self.steering_angle = min(self.steering_angle + 0.1, 0.35)
        elif action == 'BRAKE':
            self.speed = 0.0
        elif action == 'NEUTRAL':
            self.speed = 0.0
            self.steering_angle = 0.0
        elif action == 'QUIT':
            self.get_logger().info("Shutting down teleop...")
            exit()
        else:
            self.get_logger().warn(f"Unknown key: {repr(key)}")


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardAckermannController()
    thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    thread.start()

    while True:
        controller.perform_action()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
