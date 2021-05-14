from anro_msg.srv import Jint

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from rclpy.clock import ROSClock
import time


class MinimalService(Node):
    global first

    def __init__(self):
        super().__init__('minimal_service')
        if first:
            self.starting_positions = [0, 0, 0, 0]
        self.srv = self.create_service(Jint, 'interpolacja_konfiguracyjna', self.configuration_interpolation_callback)

    def configuration_interpolation_callback(self, request, response):
        self.get_logger().info('Incoming request \n j1: %f \n j2: %f \n j3: %f \n j4: %f \n time: %f \n type: %s' % (request.j1, request.j2, request.j3, request.j4, request.time, request.inttype))
        if request.inttype != 'pol' and request.inttype != 'lin':
            response.result = '\n Niepowodzenie - Nieprawidłowy rodzaj interpolacji'
            return response
        if request.time <= 0:
            response.result = '\n Niepowodzenie - Czas musi być większy od zera'
            return response

        starting_positions = self.starting_positions
        sample_time = 0.1

        if(request.inttype == 'pol'):
            self.a0 = [starting_positions[0], starting_positions[1], starting_positions[2], starting_positions[3]]
            self.a2 = [
                3*((request.j1 - starting_positions[0])/(request.time)**2),
                3*((request.j2 - starting_positions[1])/(request.time)**2),
                3*((request.j3 - starting_positions[2])/(request.time)**2),
                3*((request.j4 - starting_positions[3])/(request.time)**2)]
            self.a3 = [
                -2*((request.j1 - starting_positions[0])/(request.time)**3),
                -2*((request.j2 - starting_positions[1])/(request.time)**3),
                -2*((request.j3 - starting_positions[2])/(request.time)**3),
                -2*((request.j4 - starting_positions[3])/(request.time)**3)]

        i = 0
        while (True):
            i += 1
            if ((i)*sample_time < request.time):
                self.new_joints(i*sample_time, starting_positions, request.j1, request.j2, request.j3, request.j4, request.inttype, request.time)
                time.sleep(sample_time)
            else:
                self.new_joints(request.time, starting_positions, request.j1, request.j2, request.j3, request.j4, request.inttype, request.time)
                self.starting_positions = [request.j1, request.j2, request.j3, request.j4]
                break

        response.result = '\n Interpolacja zakończona powodzeniem'
        first = False
        return response

    def new_joints(self, current_time, starting_positions, j1, j2, j3, j4, intType, time):

        qos_profile = QoSProfile(depth=10)
        joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['base-cyl', 'dummy-arm1', 'arm1-arm2', 'arm2-arm3']

        if (intType == 'lin'):
            j1_new = starting_positions[0] + ((j1 - starting_positions[0])/time)*current_time
            j2_new = starting_positions[1] + ((j2 - starting_positions[1])/time)*current_time
            j3_new = starting_positions[2] + ((j3 - starting_positions[2])/time)*current_time
            j4_new = starting_positions[3] + ((j4 - starting_positions[3])/time)*current_time

        elif (intType == 'pol'):
            j1_new = self.a0[0] + self.a2[0]*(current_time)**2 + self.a3[0]*(current_time)**3 
            j2_new = self.a0[1] + self.a2[1]*(current_time)**2 + self.a3[1]*(current_time)**3 
            j3_new = self.a0[2] + self.a2[2]*(current_time)**2 + self.a3[2]*(current_time)**3
            j4_new = self.a0[3] + self.a2[3]*(current_time)**2 + self.a3[3]*(current_time)**3 
        
        joint_state.position = [float(j1_new), float(j2_new), float(j3_new), float(j4_new)]

        joint_pub.publish(joint_state)
        

def main(args=None):
    global first
    first = True
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()