
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TurtleSimJoyController(Node):
    def __init__(self):
        super().__init__('esp_controller')

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Scale factors for linear and angular velocity
        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.get_logger().info("TurtleSim Joy Controller Initialized!")

    def joy_callback(self, msg: Joy):
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.linear_scale
        twist.linear.y = msg.axes[2] * self.linear_scale
        twist.angular.z = msg.axes[0] * self.angular_scale

        self.velocity_publisher.publish(twist)
        self.get_logger().debug(f"Published velocity: {twist}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSimJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
