import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Publisher for control actions
        self.control_pub = self.create_publisher(Twist, '/control/action', 10)
        
        # Subscriber to localization feedback
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )
        
        # Reference position
        self.reference_x = 10.0
        self.reference_y = 5.0
        
        # Control gains (simple proportional controller)
        self.kp = 0.5
        
        self.get_logger().info('Control Node Started')
        self.get_logger().info(f'Reference position: x={self.reference_x}, y={self.reference_y}')

    def pose_callback(self, msg):
        """Receive feedback and compute control action"""
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        
        # Computing error
        error_x = self.reference_x - current_x
        error_y = self.reference_y - current_y
        
        # Simple proportional control
        control_msg = Twist()
        control_msg.linear.x = self.kp * error_x
        control_msg.linear.y = self.kp * error_y
        control_msg.linear.z = 0.0
        control_msg.angular.z = 0.0
        
        self.control_pub.publish(control_msg)
        
        self.get_logger().info(
            f'Feedback: pos=({current_x:.2f}, {current_y:.2f}), '
            f'Error=({error_x:.2f}, {error_y:.2f}), '
            f'Control=({control_msg.linear.x:.2f}, {control_msg.linear.y:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()