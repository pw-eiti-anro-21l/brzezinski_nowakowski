"""Similar to turtle_teleop_key, but control keys are set with parameters."""

import curses

import geometry_msgs.msg

import rclpy
import rclpy.node
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType

stdscr = curses.initscr()


class TurtlesimControl(rclpy.node.Node):
    """Node allowing to control turtlesim with keys set in parameters."""

    def __init__(self):
        """Initilize turtlesim_control node."""
        super().__init__('anro_turtlesim_control')
        self.publisher_ = self.create_publisher(
            geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('forward', 'y')
        self.declare_parameter('backward', 'h')
        self.declare_parameter('left', 'g')
        self.declare_parameter('right', 'j')
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

        # key = kb.purging_getch()
        key = next_key = stdscr.getch()
        while next_key != -1:
            key = next_key
            next_key = stdscr.getch()
        if key == ord(stop):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif key == ord(forward):
            msg.linear.x = 1.0
            msg.angular.z = 0.0
        elif key == ord(backward):
            msg.linear.x = -1.0
            msg.angular.z = 0.0
        elif key == ord(left):
            msg.linear.x = 0.0
            msg.angular.z = 1.0
        elif key == ord(right):
            msg.linear.x = 0.0
            msg.angular.z = -1.0
        elif key == ord(stop):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif key == -1:
            self.i += 1
            return
        self.publisher_.publish(msg)
        self.i += 1

    def introduction(self):
        """Print instructions for user."""
        stdscr.addstr('Hello world!\n')
        stdscr.addstr('Below you can see how to control you turtle:')
        stdscr.addstr(
            '\nForward: ' +
            self.get_parameter('forward').get_parameter_value().string_value
            )
        stdscr.addstr(
            '\nBackward: ' +
            self.get_parameter('backward').get_parameter_value().string_value
            )
        stdscr.addstr(
            '\nLeft: ' +
            self.get_parameter('left').get_parameter_value().string_value
            )
        stdscr.addstr(
            '\nRight: ' +
            self.get_parameter('right').get_parameter_value().string_value
            )
        if(
          self.get_parameter('stop').get_parameter_value().string_value == ' '
          ):
            stdscr.addstr('\nStop: space\n')
        else:
            stdscr.addstr(
                '\nStop: ' +
                self.get_parameter('stop').get_parameter_value().string_value +
                '\n'
                )


def main(args=None):
    """Run turtlesim_control."""
    rclpy.init(args=args)
    turtlesim_control = TurtlesimControl()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()
    turtlesim_control.introduction()
    try:
        rclpy.spin(turtlesim_control)
    except KeyboardInterrupt:
        pass
    curses.nocbreak()
    curses.echo()
    curses.endwin()
    turtlesim_control.get_logger().info('Closing anro_turtlesim_control')
    turtlesim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
