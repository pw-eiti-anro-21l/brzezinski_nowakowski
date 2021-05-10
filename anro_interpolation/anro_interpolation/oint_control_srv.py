from anro_msg.srv import Oint

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from rclpy.clock import ROSClock
from math import sin, cos
import time


class MinimalService(Node):
    global first
    global params

    def __init__(self):
        super().__init__('minimal_service')
        if first:
            self.starting_XYZ_RPY = [0, 0, 0, 0, 0, 0]
        self.srv = self.create_service(Oint, 'interpolacja_operacyjna', self.operation_interpolation_callback)

    def operation_interpolation_callback(self, request, response):
        self.get_logger().info('Incoming request \n x: %f \n y: %f \n z: %f \n roll: %f \n pitch: %f \n yaw: %f \n time: %f \n type: %s' % (request.x, request.y, request.z, request.roll, request.pitch, request.yaw, request.time, request.inttype))
        if request.inttype !== 'pol' and request.inttype !== 'lin':
            response.result = '\n Niepowodzenie - Nieprawidłowy rodzaj interpolacji'
            return response
        if request.time <= 0:
            response.result = '\n Niepowodzenie - Czas musi być większy od zera'
            return response
            
        starting_XYZ_RPY = self.starting_XYZ_RPY
        sample_time = 0.1

        if(request.inttype == 'pol'):
            self.a0 = [starting_XYZ_RPY[0], starting_XYZ_RPY[1], starting_XYZ_RPY[2], starting_XYZ_RPY[3], starting_XYZ_RPY[4], starting_XYZ_RPY[5]]
            self.a2 = [
                3*((request.x - starting_XYZ_RPY[0])/(request.time)**2),
                3*((request.y - starting_XYZ_RPY[1])/(request.time)**2),
                3*((request.z - starting_XYZ_RPY[2])/(request.time)**2),
                3*((request.roll - starting_XYZ_RPY[3])/(request.time)**2),
                3*((request.pitch - starting_XYZ_RPY[4])/(request.time)**2),
                3*((request.yaw - starting_XYZ_RPY[5])/(request.time)**2)]
            self.a3 = [
                -2*((request.x - starting_XYZ_RPY[0])/(request.time)**3),
                -2*((request.y - starting_XYZ_RPY[1])/(request.time)**3),
                -2*((request.z - starting_XYZ_RPY[2])/(request.time)**3),
                -2*((request.roll - starting_XYZ_RPY[3])/(request.time)**3),
                -2*((request.pitch - starting_XYZ_RPY[4])/(request.time)**3),
                -2*((request.yaw - starting_XYZ_RPY[5])/(request.time)**3)]

        qos_profile = QoSProfile(depth=10)
        self.path_pub = self.create_publisher(Path, '/Path', qos_profile)
        self.path = Path()
        now = self.get_clock().now()
        self.path.header.stamp = now.to_msg()
        self.path.header.frame_id = 'base'

        i = 0
        while (True):
            i += 1
            if ((i)*sample_time < request.time):
                self.new_position(i*sample_time, starting_XYZ_RPY, request.x, request.y, request.z, request.roll, request.pitch, request.yaw, request.inttype, request.time)
                time.sleep(sample_time)
            else:
                self.new_position(request.time, starting_XYZ_RPY, request.x, request.y, request.z, request.roll, request.pitch, request.yaw, request.inttype, request.time)
                self.starting_XYZ_RPY = [request.x, request.y, request.z, request.roll, request.pitch, request.yaw]
                break

        response.result = '\n Interpolacja zakończona powodzeniem'
        first = False
        return response

    def new_position(self, current_time, starting_XYZ_RPY, x, y, z, roll, pitch, yaw, intType, time):

        qos_profile = QoSProfile(depth=10)
        pose_pub = self.create_publisher(PoseStamped, '/PoseStamped', qos_profile)
        
        nodeName = self.get_name()

        pose_stamped = PoseStamped()
        point = Point()
        quaternion = Quaternion()

        now = self.get_clock().now()
        pose_stamped.header.stamp = now.to_msg()
        pose_stamped.header.frame_id = 'base'

        if (intType == 'lin'):
            x_new = starting_XYZ_RPY[0] + ((x - starting_XYZ_RPY[0])/time)*current_time
            y_new = starting_XYZ_RPY[1] + ((y - starting_XYZ_RPY[1])/time)*current_time
            z_new = starting_XYZ_RPY[2] + ((z - starting_XYZ_RPY[2])/time)*current_time
            roll_new = starting_XYZ_RPY[3] + ((roll - starting_XYZ_RPY[3])/time)*current_time
            pitch_new = starting_XYZ_RPY[4] + ((pitch - starting_XYZ_RPY[4])/time)*current_time
            yaw_new = starting_XYZ_RPY[5] + ((yaw - starting_XYZ_RPY[5])/time)*current_time
        elif (intType == 'pol'):
            x_new = self.a0[0] + self.a2[0]*(current_time)**2 + self.a3[0]*(current_time)**3 
            y_new = self.a0[1] + self.a2[1]*(current_time)**2 + self.a3[1]*(current_time)**3 
            z_new = self.a0[2] + self.a2[2]*(current_time)**2 + self.a3[2]*(current_time)**3
            roll_new = self.a0[3] + self.a2[3]*(current_time)**2 + self.a3[3]*(current_time)**3
            pitch_new = self.a0[4] + self.a2[4]*(current_time)**2 + self.a3[4]*(current_time)**3 
            yaw_new = self.a0[5] + self.a2[5]*(current_time)**2 + self.a3[5]*(current_time)**3  

        point.x = x_new
        point.y = y_new
        point.z = z_new

        pose_stamped.pose.position = point
        pose_stamped.pose.orientation = self.euler_to_quaternion(roll_new, pitch_new, yaw_new)
        self.path.poses.append(pose_stamped)

        pose_pub.publish(pose_stamped)
        self.path_pub.publish(self.path)
                
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    global first
    first = True

    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()