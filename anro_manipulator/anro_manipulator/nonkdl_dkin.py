#! /usr/bin/env python
from math import sin, cos, pi
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
            super().__init__('nonkdl_dkin')

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
            self.al = params[9]
            self.t3 = -params[11]
            self.a3 = params[12]
            self.t4 = params[15]
            self.a4 = params[16]
            self.t5 = params[19]
            self.a5 = params[20]

            self.f1 = 0.0
            self.f3 = 0.0
            self.f4 = 0.0
            self.f5 = 0.0

            # Deklaracje typu sub i pub
            qos_profile = QoSProfile(depth=10)
            self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
            self.subscription
            self.pose_pub = self.create_publisher(PoseStamped, 'PoseStamped2', qos_profile)
            self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
            self.nodeName = self.get_name()
            self.get_logger().info("{0} started".format(self.nodeName))
            
            degree = pi / 180.0
            loop_rate = self.create_rate(30)

            # message declarations
            pose_stamped = PoseStamped()
            point = Point()
            quaternion = Quaternion()

            try:
                while True:
                    rclpy.spin_once(self)

                    # Tworzenie nagłówka
                    now = self.get_clock().now()
                    pose_stamped.header.stamp = now.to_msg()
                    pose_stamped.header.frame_id = 'base'

                    # Obliczanie rotacji
                    roll = 0 
                    pitch = self.f3 + self.f4 + self.f5
                    yaw = self.f1 

                    # Obliczanie położenia
                    point.x = self.a4*(cos(self.f4)*(cos(self.f1)*cos(self.f3) - cos(self.al)*sin(self.f1)*sin(self.f3)) - sin(self.f4)*(cos(self.f1)*sin(self.f3) + cos(self.al)*cos(self.f3)*sin(self.f1))) + self.a3*(cos(self.f1)*cos(self.f3) - cos(self.al)*sin(self.f1)*sin(self.f3)) + self.a5*(cos(self.f5)*(cos(self.f4)*(cos(self.f1)*cos(self.f3) - cos(self.al)*sin(self.f1)*sin(self.f3)) - sin(self.f4)*(cos(self.f1)*sin(self.f3) + cos(self.al)*cos(self.f3)*sin(self.f1))) - sin(self.f5)*(cos(self.f4)*(cos(self.f1)*sin(self.f3) + cos(self.al)*cos(self.f3)*sin(self.f1)) + sin(self.f4)*(cos(self.f1)*cos(self.f3) - cos(self.al)*sin(self.f1)*sin(self.f3))))
                    point.y = self.a4*(cos(self.f4)*(cos(self.f3)*sin(self.f1) + cos(self.al)*cos(self.f1)*sin(self.f3)) - sin(self.f4)*(sin(self.f1)*sin(self.f3) - cos(self.al)*cos(self.f1)*cos(self.f3))) + self.a3*(cos(self.f3)*sin(self.f1) + cos(self.al)*cos(self.f1)*sin(self.f3)) + self.a5*(cos(self.f5)*(cos(self.f4)*(cos(self.f3)*sin(self.f1) + cos(self.al)*cos(self.f1)*sin(self.f3)) - sin(self.f4)*(sin(self.f1)*sin(self.f3) - cos(self.al)*cos(self.f1)*cos(self.f3))) - sin(self.f5)*(cos(self.f4)*(sin(self.f1)*sin(self.f3) - cos(self.al)*cos(self.f1)*cos(self.f3)) + sin(self.f4)*(cos(self.f3)*sin(self.f1) + cos(self.al)*cos(self.f1)*sin(self.f3))))
                    point.z = self.d1 + self.d2 + self.a5*(cos(self.f5)*(cos(self.f3)*sin(self.al)*sin(self.f4) + cos(self.f4)*sin(self.al)*sin(self.f3)) + sin(self.f5)*(cos(self.f3)*cos(self.f4)*sin(self.al) - sin(self.al)*sin(self.f3)*sin(self.f4))) + self.a4*(cos(self.f3)*sin(self.al)*sin(self.f4) + cos(self.f4)*sin(self.al)*sin(self.f3)) + self.a3*sin(self.al)*sin(self.f3)

                    # Uzupełnienie wiadomości i wysłanie jej                  
                    pose_stamped.pose.position = point
                    pose_stamped.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                    self.pose_pub.publish(pose_stamped)

            except KeyboardInterrupt:
                    pass

    def listener_callback(self, msg):
        # Uaktualnienie kątów
        [j1, j2, j3, j4] = msg.position
        self.f1 = j1 + self.t1
        self.f3 = j2 + self.t3
        self.f4 = j3 + self.t4
        self.f5 = j4 + self.t5

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():    
    node = StatePublisher()

if __name__ == '__main__':
    main()
