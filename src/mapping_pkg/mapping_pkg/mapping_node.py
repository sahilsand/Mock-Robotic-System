import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Publisher for map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Subscriber to camera sensor data
        self.sensor_sub = self.create_subscription(
            Image,
            '/sensor/data',
            self.sensor_callback,
            10
        )
        
        # Map update counter
        self.update_count = 0
        
        self.get_logger().info('Mapping Node Started')

    def sensor_callback(self, msg):
        """Process sensor data and update map"""
        self.get_logger().info(f'Received sensor data for mapping: {msg.width}x{msg.height}')
        
        # Simulating map building
        # In real system - process image, extract obstacles, update occupancy grid
        self.update_count += 1
        
        # Publishing updated map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 0.05  # 5cm per cell
        map_msg.info.width = 100
        map_msg.info.height = 100
        # Initializing empty map (0 = free space)
        map_msg.data = [0] * (100 * 100)
        
        self.map_pub.publish(map_msg)
        self.get_logger().info(f'Updated Map (iteration {self.update_count})')

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()