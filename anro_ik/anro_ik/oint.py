import sys

from anro_msg.srv import Ointik

import rclpy
from rclpy.node import Node
from math import pi


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Ointik, 'interpolacja_operacyjna2')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Ointik.Request()

    def send_request(self):
        try:
            if(float(sys.argv[1]) <= 0):
                self.get_logger().info('Niepoprawna wartość a')
                raise ValueError()
            else:
                self.req.a = float(sys.argv[1])
            if(float(sys.argv[2]) <= 0):
                self.get_logger().info('Niepoprawna wartość b')
                raise ValueError()
            else:
                self.req.b = float(sys.argv[2])  
            if(float(sys.argv[3]) <= 0):
                self.get_logger().info('Niepoprawna wartość czasu')
                raise ValueError()
            else:
                self.req.time= float(sys.argv[3])
            if(str(sys.argv[4]) !='rec' and str(sys.argv[4]) !='ell'):
                self.get_logger().info('Błędy rodzaj trajektorii')
                raise ValueError()
            else:
                self.req.type = (sys.argv[4])  
        except IndexError:
            print("\n Podano za mało parametrów")
            raise Exception()
        except ValueError:
            print("\n Podane parametry są niewłaściwe")
            raise Exception()
            
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    try:
        minimal_client = MinimalClientAsync()
        minimal_client.send_request()
    except:
        print("\n Wystąpił błąd")
    else:
        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info(
                        '\n Wynik interpolacji: %s' %
                        (response.result))
                break
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()