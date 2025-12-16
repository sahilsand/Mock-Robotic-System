import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(PoseStamped, '/localization/pose', 10)
        
        # Subscriber to camera sensor data
        self.sensor_sub = self.create_subscription(
            Image,
            '/sensor/data',
            self.sensor_callback,
            10
        )
        
        # Simulated position that changes over time
        self.x_position = 0.0
        self.y_position = 0.0
        
        self.get_logger().info('Localization Node Started')

    def sensor_callback(self, msg):
        """Process sensor data and estimate position"""
        # Simulating localization based on sensor data
        # In real system - process image, extract features, estimate pose
        self.get_logger().info(f'Received sensor data: {msg.width}x{msg.height}')
        
        # Simulating position estimation (simple increment for demo)
        self.x_position += 0.1
        self.y_position += 0.05
        
        # Publishing estimated pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x_position
        pose_msg.pose.position.y = self.y_position
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published Estimated Pose: x={self.x_position:.2f}, y={self.y_position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()