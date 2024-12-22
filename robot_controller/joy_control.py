import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to joystick messages
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Joystick axis mapping
        self.linear_axis = 1  # Left stick vertical for linear velocity
        self.angular_axis = 0  # Left stick horizontal for angular velocity

        # Threshold for dead zone
        self.dead_zone = 0.1  # Joystick values below this are considered neutral

        self.get_logger().info('Joystick Controller Node Initialized.')

    def scale_to_range(self, value):
        """
        Scale joystick input from [-1, 1] to [1, -1].
        If within the dead zone, return 0.
        """
        if abs(value) < self.dead_zone:
            return 0.0  # Neutral position
        return value

    def joy_callback(self, msg: Joy):
        # Create a Twist message
        twist = Twist()

        # Scale joystick input
        twist.linear.x = self.scale_to_range(msg.axes[self.linear_axis])
        twist.angular.z = self.scale_to_range(msg.axes[self.angular_axis])

        # Publish the velocity command
        self.publisher.publish(twist)

        # Log the values for debugging
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    joystick_controller = JoystickController()

    # Spin the node
    rclpy.spin(joystick_controller)

    # Shutdown the node
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
