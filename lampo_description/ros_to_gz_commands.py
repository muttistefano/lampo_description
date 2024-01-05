#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RosToGz(Node):

    def __init__(self):
        super().__init__('ros_to_gz_commands')
        self.pj1_ = self.create_publisher(Float64, 'shoulder_pan_joint_cmd', 1)
        self.pj2_ = self.create_publisher(Float64, 'shoulder_lift_joint_cmd', 1)
        self.pj3_ = self.create_publisher(Float64, 'elbow_joint_cmd', 1)
        self.pj4_ = self.create_publisher(Float64, 'wrist_1_joint_cmd', 1)
        self.pj5_ = self.create_publisher(Float64, 'wrist_2_joint_cmd', 1)
        self.pj6_ = self.create_publisher(Float64, 'wrist_3_joint_cmd', 1)
        self.sub_ = self.create_subscription(JointState,'joint_commands',self.listener_callback,1)

        self.j1   = Float64() 
        self.j2   = Float64() 
        self.j3   = Float64() 
        self.j4   = Float64() 
        self.j5   = Float64() 
        self.j6   = Float64() 

    def listener_callback(self, msg:JointState):
        self.j1.data = msg.position[0]
        self.j2.data = msg.position[1]
        self.j3.data = msg.position[2]
        self.j4.data = msg.position[3]
        self.j5.data = msg.position[4]
        self.j6.data = msg.position[5]
        self.pj1_.publish(self.j1)
        self.pj2_.publish(self.j2)
        self.pj3_.publish(self.j3)
        self.pj4_.publish(self.j4)
        self.pj5_.publish(self.j5)
        self.pj6_.publish(self.j6)


def main(args=None):
    rclpy.init(args=args)
    RTG = RosToGz()
    rclpy.spin(RTG)
    rclpy.shutdown()


if __name__ == '__main__':
    main()