from anro_msg.srv import Ointik

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from rclpy.clock import ROSClock
from math import sin, cos, sqrt, pi
import time


class MinimalService(Node):
    global first
    global params

    def __init__(self):
        super().__init__('minimal_service')
        if first:
            self.starting_XYZ_RPY = [0.4, 0.0, 0.4, 0, 0, 0]
        self.srv = self.create_service(Ointik, 'interpolacja_operacyjna2', self.operation_interpolation_callback)

    def operation_interpolation_callback(self, request, response):
        self.get_logger().info('Incoming request \n a: %f \n b: %f \n time: %f \n type: %s' % (request.a, request.b, request.time, request.type))
            
        starting_XYZ_RPY = self.starting_XYZ_RPY
        sample_time = 0.1

        qos_profile = QoSProfile(depth=10)
        self.path_pub = self.create_publisher(Path, '/Path', qos_profile)
        self.path = Path()
        now = self.get_clock().now()
        self.path.header.stamp = now.to_msg()
        self.path.header.frame_id = 'base'

        a = request.a
        b = request.b
        
        if(request.type == 'rec'):

            ratio = a/(a+b)
            timer_1 = ratio*request.time/2
            timer_2 = (1-ratio)*request.time/2
            timer_3 = (1-ratio)*request.time/2
            timer_4 = ratio*request.time/2
            timers = [timer_1, timer_2, timer_3, timer_4]
            
            for index, timer in enumerate(timers):
                i = 0
                while (True):
                    i += 1
                    qos_profile = QoSProfile(depth=10)
                    pose_pub = self.create_publisher(PoseStamped, '/PoseStamped', qos_profile)
                    
                    nodeName = self.get_name()

                    pose_stamped = PoseStamped()

                    now = self.get_clock().now()
                    pose_stamped.header.stamp = now.to_msg()
                    pose_stamped.header.frame_id = 'base'

                    if ((i)*sample_time < timer):
                        pose_stamped.pose.position.x = starting_XYZ_RPY[0]
                        pose_stamped.pose.position.y = starting_XYZ_RPY[1]
                        pose_stamped.pose.position.z = starting_XYZ_RPY[2]
                        
                        if index == 0:
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2] + (a/timer)*i*sample_time
                        elif index == 1:
                            self.get_logger().info('Incoming request \n timer-timer_1: %f \n timer-timer_1: %f:' % (b/timer-timer_1, i*sample_time))
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2] + a
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1] + (b/timer)*(i*sample_time)
                        elif index == 2:
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2] + a - (a/timer)*i*sample_time
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1] + b
                        elif index == 3:
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1] + b - (b/timer)*i*sample_time

                        self.path.poses.append(pose_stamped)
                        pose_pub.publish(pose_stamped)
                        self.path_pub.publish(self.path)
                        time.sleep(sample_time)
                    else:
                        pose_stamped.pose.position.x = starting_XYZ_RPY[0]
                        pose_stamped.pose.position.y = starting_XYZ_RPY[1]
                        pose_stamped.pose.position.z = starting_XYZ_RPY[2]
                        
                        if index == 0:
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2] + a
                        elif index == 1:
                            self.get_logger().info('Incoming request \n timer-timer_1: %f \n timer-timer_1: %f:' % (b/timer-timer_1, i*sample_time))
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2] + a
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1] + b
                        elif index == 2:
                            pose_stamped.pose.position.z = starting_XYZ_RPY[2]
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1] + b
                        elif index == 3:
                            pose_stamped.pose.position.y = starting_XYZ_RPY[1]

                        self.path.poses.append(pose_stamped)
                        pose_pub.publish(pose_stamped)
                        self.path_pub.publish(self.path)
                        time.sleep(sample_time)
                        break
            
        if request.type == 'ell':
            i = 0
            while (True):
                i += 1
                qos_profile = QoSProfile(depth=10)
                pose_pub = self.create_publisher(PoseStamped, '/PoseStamped', qos_profile)

                nodeName = self.get_name()

                pose_stamped = PoseStamped()

                now = self.get_clock().now()
                pose_stamped.header.stamp = now.to_msg()
                pose_stamped.header.frame_id = 'base'

                if ((i)*sample_time < request.time):
                    pose_stamped.pose.position.z = starting_XYZ_RPY[0]

                    pose_stamped.pose.position.y = starting_XYZ_RPY[1] + a*cos(2*pi*(1/request.time)*sample_time*i) - a
                    pose_stamped.pose.position.x = starting_XYZ_RPY[2] + b*sin(2*pi*(1/request.time)*sample_time*i) 

                    self.path.poses.append(pose_stamped)
                    pose_pub.publish(pose_stamped)
                    self.path_pub.publish(self.path)
                    time.sleep(sample_time)

                else:
                    pose_stamped.pose.position.x = starting_XYZ_RPY[0]
                    pose_stamped.pose.position.y = starting_XYZ_RPY[1]
                    pose_stamped.pose.position.z = starting_XYZ_RPY[2]

                    self.path.poses.append(pose_stamped)
                    pose_pub.publish(pose_stamped)
                    self.path_pub.publish(self.path)
                    time.sleep(sample_time)
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