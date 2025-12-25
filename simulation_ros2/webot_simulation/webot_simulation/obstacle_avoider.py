import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive
import sensor_msgs_py.point_cloud2 as pc2

class LidarAvoider(Node):
    def __init__(self):
        super().__init__('lidar_avoider')

        self.publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/TT02_jaune/RpLidarA2/point_cloud',
            self.lidar_callback,
            10
        )
        self.get_logger().info('LidarAvoider node started, watching for obstacles...')

    def lidar_callback(self, msg):
        min_distance = float('inf')

        # Parse point cloud
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            distance = (x**2 + y**2)**0.5

            # Check if point is in front (e.g., within -0.2 < y < 0.2 and 0 < x < 1)
            if -0.2 < y < 0.2 and 0 < x < 1.0:
                if distance < min_distance:
                    min_distance = distance

        cmd = AckermannDrive()

        # Decision logic
        if min_distance < 0.4:
            self.get_logger().info(f"🚨 Obstacle detected at {min_distance:.2f}m — avoiding!")
            cmd.speed = -0.5  # Go backward
            cmd.steering_angle = -0.3  # Turn right
        else:
            cmd.speed = 1.0
            cmd.steering_angle = 0.0

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
