import getch

import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import String
import geometry_msgs.msg #import Twist

class MinimalPublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('ANRO_Turtlesim_control')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('my_parameter', 'c')
        self.i = 0

    def timer_callback(self):        
        msg = geometry_msgs.msg.Twist()
        
        key = getch.getch()
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        if  key == my_param:
            msg.linear.x = 1.0
            msg.angular.z = 0.6
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('key code: %d' % ord(key))
        
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
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()