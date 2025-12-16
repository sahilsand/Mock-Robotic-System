import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CameraSensorNode(Node):
    def __init__(self):
        super().__init__('camera_sensor_node')
        self.pub = self.create_publisher(Image, '/sensor/data', 10)
        self.create_timer(1.0, self.publish_callback)
        self.get_logger().info('Camera Sensor Node Started')

    def publish_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = "mono8"
        msg.step = 640
        # Simulating some sensor data (e.g gradient pattern)
        data = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        msg.data = data.tobytes()
        self.pub.publish(msg)
        self.get_logger().info('Published Image Data')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()