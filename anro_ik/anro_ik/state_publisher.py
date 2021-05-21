#! /usr/bin/env python
from math import sin, cos, pi, sqrt, atan2, acos, copysign, asin
import threading
import rclpy
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped
from ament_index_python.packages import get_package_share_directory
class StatePublisher(Node):

    def __init__(self):
            rclpy.init()
            super().__init__('state_publisher')

            # wczytanie tabeli DH
            package_share_directory = get_package_share_directory('anro_manipulator')
            with open(package_share_directory + '/manipulator.json') as json_file:
                matrix = json.load(json_file)

            # with open('./urdf/manipulator.json') as json_file:
            #     matrix = json.load(json_file)

            # pobranie wszystkich wartości z tabeli DH
            params = []
            for row in matrix:
                params.append(row['a'])
                params.append(row['alfa'] * pi / 180)
                params.append(row['d'])
                params.append(row['theta'] * pi / 180)

            # przypisanie interesujących nas wartości       
            self.d1 = params[2]
            self.t1 = params[3] 
            self.d2 = params[6] 
            self.a2 = params[9]
            self.t3 = params[11]
            self.r3 = params[12]
            self.t4 = params[15]
            self.r4 = params[16]
            self.t5 = params[19]
            self.r5 = params[20]

            self.f1 = 0.0
            self.f3 = 0.0
            self.f4 = 0.0
            self.f5 = 0.0


            qos_profile = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
            self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
            self.nodeName = self.get_name()


            
            self.subscription = self.create_subscription(PoseStamped, '/PoseStamped',self.listener_callback, 4)
            self.subscription  # prevent unused variable warning



            self.get_logger().info("{0} started".format(self.nodeName))
            self.x = 0
            self.y = 0
            self.z = 0
            self.pitch = 0
            degree = pi / 180.0
            loop_rate = self.create_rate(30)

            # robot state
            i = 0
            x = 0.
            y = 0.
            z = 0.
            r = 0.

            # message declarations
            odom_trans = TransformStamped()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'axis'
            joint_state = JointState()

            try:
                while rclpy.ok():
                    x = self.x
                    y = self.y
                    z = self.z
                    r = -self.pitch
                    z2 = z - self.r5*sin(r) - self.d1 - self.d2
                    l = sqrt(x*x + y*y)
                    l2 = l -  self.r5*cos(r)
                    c_2 = l2*l2 + z2 * z2
                    
                    if c_2  < (self.r3 + self.r4)**2: #Check if position is achievable
                        f1 = atan2(y, x)
                        fi4a = pi - acos( (c_2 - self.r3 * self.r3 - self.r4 * self.r4) / (-2 * self.r3 * self.r4) )
                        fi4b = - fi4a
                        _fi3_1 =  acos( (self.r4*self.r4 - self.r3 * self.r3 - c_2) / (-2 * self.r3 * sqrt(c_2)) ) 
                        _fi3_2 =  atan2(z2, l2)
                        fi3a= -_fi3_2 - _fi3_1
                        fi3b= -_fi3_2 + _fi3_1

                        fi5a = -r - fi4a - fi3a
                        fi5b = -r - fi4b - fi3b

                        # update joint_state
                        now = self.get_clock().now()
                        joint_state.header.stamp = now.to_msg()
                        joint_state.name = ['base-cyl', 'dummy-arm1', 'arm1-arm2', 'arm2-arm3']
                        joint_state.position = [f1, fi3a, fi4a,  fi5a]

                        # self.get_logger().info('Publishing: "%s"' % joint_state.position)
                        self.joint_pub.publish(joint_state)
                        self.broadcaster.sendTransform(odom_trans)

                    # loop_rate.sleep()
                    rclpy.spin_once(self)

            except KeyboardInterrupt:
                    pass

    def listener_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        #pitch (y-axis rotation)
        qz = 2 * (msg.pose.orientation.x * msg.pose.orientation.z + msg.pose.orientation.y * msg.pose.orientation.w)
        qx = 1 - 2 * (msg.pose.orientation.y * msg.pose.orientation.y + msg.pose.orientation.z * msg.pose.orientation.z)
        self.pitch = atan2(qz, qx)
        self.get_logger().info('I heard: x: %1.2f, y: %1.2f, z: %1.2f, p: %3f' % (self.x,self.y,self.z,self.pitch*180/pi))
def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()