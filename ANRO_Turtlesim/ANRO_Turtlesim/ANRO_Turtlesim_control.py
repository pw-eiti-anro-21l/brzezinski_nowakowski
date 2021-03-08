import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import geometry_msgs.msg #import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('ANRO_Turtlesim_control')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.6
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()