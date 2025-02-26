import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.subscription

    def joy_callback(self, msg):
        # The Joy message contains axes and buttons info from the joystick
        x_axis = msg.axes[0]
        y_axis = msg.axes[1]
        self.get_logger().info(f"x axis: {x_axis}")
        self.get_logger().info(f"y axis: {y_axis}")

def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
