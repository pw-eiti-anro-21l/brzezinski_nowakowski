"""Similar to turtle_teleop_key, but control keys are set with parameters."""

from anro_turtlesim.kbhit import KBHit

import geometry_msgs.msg

import rclpy
import rclpy.node
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType

kb = KBHit()


class TurtlesimControl(rclpy.node.Node):
    """Node allowing to control turtlesim with keys set in parameters."""

    def __init__(self):
        """Initilize turtlesim_control node."""
        super().__init__('anro_turtlesim_control')
        self.publisher_ = self.create_publisher(
            geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('forward', 'w')
        self.declare_parameter('backward', 's')
        self.declare_parameter('left', 'd')
        self.declare_parameter('right', 'a')
        self.declare_parameter('stop', ' ')
        self.i = 0

    def timer_callback(self):
        """Handle turtlesim_control loop."""
        forward = \
            self.get_parameter('forward').get_parameter_value().string_value
        backward = \
            self.get_parameter('backward').get_parameter_value().string_value
        left = self.get_parameter('left').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        stop = self.get_parameter('stop').get_parameter_value().string_value
        msg = geometry_msgs.msg.Twist()

        key = kb.purging_getch()
        if key == forward:
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
        elif key == stop:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # my_new_param = rclpy.parameter.Parameter(
        #     'my_parameter',
        #     rclpy.Parameter.Type.STRING,
        #     'c'
        # )
        # all_new_parameters = [my_new_param]
        # self.set_parameters(all_new_parameters)
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    """Run turtlesim_control."""
    rclpy.init(args=args)
    turtlesim_control = TurtlesimControl()
    try:
        rclpy.spin(turtlesim_control)
    except KeyboardInterrupt:
        turtlesim_control.get_logger().info('Closing anro_turtlesim_control')
    turtlesim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
