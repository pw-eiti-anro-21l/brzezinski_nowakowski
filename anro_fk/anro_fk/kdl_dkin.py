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
from PyKDL import *
import yaml

class StatePublisher(Node):

    def __init__(self):

            rclpy.init()
            super().__init__('kdl_dkin')

            self.joint_positions=JntArray(4) # utworzenie pojemnika na przychodzące kąty

            # wczytanie pliku yaml
            package_share_directory = get_package_share_directory('anro_manipulator')
            with open(package_share_directory + '/manipulator.yaml', 'r') as yaml_file:
                coords = yaml.load(yaml_file, Loader=yaml.FullLoader)
            
            xyz1 = self.convert_to_floats(coords["joint1"][0]) # potrzebuje joint 1 i 2 dla pierwszego d
            xyz11 = self.convert_to_floats(coords["joint2"][0]) # potrzebuje joint 1 i 2 dla pierwszego d
            rpy1 = self.convert_to_floats(coords["joint1"][1])
            xyz2 = self.convert_to_floats(coords["joint3"][0])
            rpy2 = self.convert_to_floats(coords["joint3"][1])
            xyz3 = self.convert_to_floats(coords["joint4"][0])
            rpy3 = self.convert_to_floats(coords["joint4"][1])
            xyz4 = self.convert_to_floats(coords["joint5"][0])
            rpy4 = self.convert_to_floats(coords["joint5"][1])
            xyz5 = self.convert_to_floats(coords["joint6"][0])
            rpy5 = self.convert_to_floats(coords["joint6"][1])

            # deklarowanie sub i pub
            qos_profile = QoSProfile(depth=10)
            self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
            self.subscription
            self.pose_pub = self.create_publisher(PoseStamped, 'PoseStamped', qos_profile)
            self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
            self.nodeName = self.get_name()
            self.get_logger().info("{0} started".format(self.nodeName))
            
            degree = pi / 180.0
            loop_rate = self.create_rate(30)
            
            # tworzenie łańcucha kinematycznego
            chain = Chain()

            base_cyl = Joint(Joint.RotZ)
            frame1 = Frame(Rotation.RPY(0,rpy2[1],rpy1[2]), Vector(0, 0, xyz1[2]+xyz11[2]))
            segment1 = Segment(base_cyl, frame1)
            chain.addSegment(segment1)

            arm1 = Joint(Joint.RotY)
            frame2 = Frame(Rotation.RPY(0, 0, rpy3[2]), Vector(xyz3[0],0,0))
            segment2 = Segment(arm1, frame2)
            chain.addSegment(segment2)
            
            arm2 = Joint(Joint.RotY)
            frame3 = Frame(Rotation.RPY(0, rpy4[2], 0), Vector(xyz4[0], 0, 0))
            segment3 = Segment(arm2, frame3)
            chain.addSegment(segment3)

            arm3 = Joint(Joint.RotY)
            frame4 = Frame(Rotation.RPY(rpy2[0], rpy5[2]+rpy2[1], rpy2[2]), Vector(xyz5[0], 0, 0))
            segment4 = Segment(arm3, frame4)
            chain.addSegment(segment4)

            # message declarations
            pose_stamped = PoseStamped()
            point = Point()
            quaternion = Quaternion()

            try:
                while True:
                    rclpy.spin_once(self)

                    # uruchomienie solvera
                    fk = ChainFkSolverPos_recursive(chain)
                    finalFrame = Frame()
                    fk.JntToCart(self.joint_positions, finalFrame)

                    # header do wiadomosci
                    now = self.get_clock().now()
                    pose_stamped.header.stamp = now.to_msg()
                    pose_stamped.header.frame_id = 'base'

                    # uzupełnianie podwiadomości
                    [point.x, point.y, point.z] = finalFrame.p
                    [quaternion.x, quaternion.y, quaternion.z, quaternion.w] = finalFrame.M.GetQuaternion()
                    
                    # uzupełnianie i wysłanie wiadomości ostatecznej
                    pose_stamped.pose.position = point
                    pose_stamped.pose.orientation = quaternion
                    self.pose_pub.publish(pose_stamped)

            except KeyboardInterrupt:
                    pass

    def listener_callback(self, msg): 
        # uaktualnienie pozycji kątów
        [j1, j2, j3, j4] = msg.position
        self.joint_positions[0]= j1
        self.joint_positions[1]= j2
        # self.get_logger().info("joint3: {0} ".format(j2))
        self.joint_positions[2]= j3
        self.joint_positions[3]= j4

    def convert_to_floats(self, string):
        string_values = string.split()
        floats=[]
        for value in string_values:
            floats.append(float(value))
        return floats

def main():    
    node = StatePublisher()

if __name__ == '__main__':
    main()
