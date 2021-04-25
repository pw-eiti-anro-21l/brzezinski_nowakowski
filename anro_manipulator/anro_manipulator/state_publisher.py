#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
class StatePublisher(Node):

    def __init__(self):
            rclpy.init()
            super().__init__('state_publisher')

            qos_profile = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
            self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
            self.nodeName = self.get_name()
            self.get_logger().info("{0} started".format(self.nodeName))
            self.declare_parameter('cyl', 90)
            self.declare_parameter('arm1', -70)
            self.declare_parameter('arm2', 30)
            self.declare_parameter('arm3', 40)

            degree = pi / 180.0
            loop_rate = self.create_rate(30)

            # robot state
            i = 0
            cyl = 0.
            arm1 = 0.
            arm2 = 0.
            arm3 = 0.

            # message declarations
            odom_trans = TransformStamped()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'axis'
            joint_state = JointState()

            try:
                while rclpy.ok():
                    cyl = float(self.get_parameter('cyl').get_parameter_value().integer_value)*pi/180
                    arm1 = float(self.get_parameter('arm1').get_parameter_value().integer_value)*pi/180
                    arm2 = float(self.get_parameter('arm2').get_parameter_value().integer_value)*pi/180
                    arm3 = float(self.get_parameter('arm3').get_parameter_value().integer_value)*pi/180
                    rclpy.spin_once(self)

                    # update joint_state
                    now = self.get_clock().now()
                    joint_state.header.stamp = now.to_msg()
                    joint_state.name = ['base-cyl', 'dummy-arm1', 'arm1-arm2', 'arm2-arm3']
                    joint_state.position = [cyl, arm1, arm2, arm3]

                    self.get_logger().info('Publishing: "%s"' % joint_state.position)
                    self.joint_pub.publish(joint_state)
                    self.broadcaster.sendTransform(odom_trans)

                    # loop_rate.sleep()

            except KeyboardInterrupt:
                    pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()