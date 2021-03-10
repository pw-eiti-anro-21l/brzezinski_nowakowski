import getch

import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import String
import geometry_msgs.msg #import Twist

class MinimalPublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('anro_turtlesim_control')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('forward', 'w')
        self.declare_parameter('backward', 's')
        self.declare_parameter('left', 'd')
        self.declare_parameter('right', 'a')
        self.i = 0

    def timer_callback(self):
        forward = self.get_parameter('forward').get_parameter_value().string_value
        backward = self.get_parameter('backward').get_parameter_value().string_value
        left = self.get_parameter('left').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        msg = geometry_msgs.msg.Twist()
        key = getch.getch()
        
        if  key == forward:
            msg.linear.x = 1.0
            msg.angular.z = 0.0
        elif key == backward:
            msg.linear.x = -1.0
            msg.angular.z = 0.0
        elif key == left:
            msg.linear.x = 0.0
            msg.angular.z = -1.0
        elif key == right:
            msg.linear.x = 0.0
            msg.angular.z = 1.0
        
        #my_new_param = rclpy.parameter.Parameter(
        #    'my_parameter',
        #    rclpy.Parameter.Type.STRING,
        #    'c'
        #)
        #all_new_parameters = [my_new_param]
        #self.set_parameters(all_new_parameters)
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info("Closing anro_turtlesim_control")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()