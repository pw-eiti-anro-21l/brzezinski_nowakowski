import sys

from anro_msg.srv import Jint

import rclpy
from rclpy.node import Node
from math import pi


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Jint, 'interpolacja_konfiguracyjna')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Jint.Request()

    def send_request(self):
        try:
            if(float(sys.argv[1])>180 or float(sys.argv[1]) < -180):
                self.get_logger().info('\n Staw 1: Podana wartość nie mieści się w zalresie roboczym') 
                raise ValueError()
            else:
                self.req.j1 = float(sys.argv[1])*pi/180

            if(float(sys.argv[2])>180 or float(sys.argv[2]) < -180):
                self.get_logger().info('\n Staw 2: Podana wartość nie mieści się w zalresie roboczym')                
                raise ValueError()
            else:
                self.req.j2= float(sys.argv[2])*pi/180

            if(float(sys.argv[3])>180 or float(sys.argv[3]) < -180):
                self.get_logger().info('\n Staw 3: Podana wartość nie mieści się w zalresie roboczym')
                raise ValueError()
            else:
                self.req.j3 = float(sys.argv[3])*pi/180
            
            if(float(sys.argv[4])>180 or float(sys.argv[4]) < -180):
                self.get_logger().info('\n Staw 4: Podana wartość nie mieści się w zalresie roboczym')
                raise ValueError()
            else:
                self.req.j4 = float(sys.argv[4])*pi/180

            if(float(sys.argv[5])<=0):
                self.get_logger().info('\n Podany czas musi być większy od zera')
                raise ValueError()
            else:
                self.req.time = float(sys.argv[5])

            if(str(sys.argv[6]) !='lin' and str(sys.argv[6]) !='pol'):
                self.get_logger().info('\n Podany typ interpolacji nie został znaleziony')
                raise ValueError()
            else:
                self.req.inttype = (sys.argv[6])  
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