import os

import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import String
import geometry_msgs.msg #import Twist


# Windows
if os.name == 'nt':
    import msvcrt

# Posix (Linux, OS X)
else:
    import sys
    import termios
    import atexit
    from select import select


class KBHit:

    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''

        if os.name == 'nt':
            pass

        else:

            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)


    def set_normal_term(self):
        ''' Resets to normal terminal.  On Windows this is a no-op.
        '''

        if os.name == 'nt':
            pass

        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)


    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
            Should not be called in the same program as getarrow().
        '''

        s = ''

        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')

        else:
            return sys.stdin.read(1)


    def getarrow(self):
        ''' Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        '''

        if os.name == 'nt':
            msvcrt.getch() # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]

        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]

        return vals.index(ord(c.decode('utf-8')))
    def  purging_getch(self):
        key = chr(0)
        while kb.kbhit():
            key = kb.getch()
        return key


    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        if os.name == 'nt':
            return msvcrt.kbhit()

        else:
            dr,dw,de = select([sys.stdin], [], [], 0)
            return dr != []
kb = KBHit()

class MinimalPublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('anro_turtlesim_control')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
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
        
        key = kb.purging_getch()
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
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
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