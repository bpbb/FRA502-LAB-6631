#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class turtlesim_pose(Node):
    def __init__(self):
        super().__init__('odom_pub')

        self.robot_pose_1 = np.array([0.0, 0.0, 0.0])
        self.robot_pose_2 = np.array([0.0, 0.0, 0.0])

        self.odom_publisher_1 = self.create_publisher(Odometry, '/odom1', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback_1, 10)

        self.odom_publisher_2 = self.create_publisher(Odometry, '/odom2', 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback_2, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback_1(self, msg):
        self.robot_pose_1[0] = msg.x
        self.robot_pose_1[1] = msg.y
        self.robot_pose_1[2] = msg.theta
        self.odom_publish('turtle1', self.robot_pose_1)

    def pose_callback_2(self, msg):
        self.robot_pose_2[0] = msg.x
        self.robot_pose_2[1] = msg.y
        self.robot_pose_2[2] = msg.theta
        self.odom_publish('turtle2', self.robot_pose_2)

    
    def odom_publish(self, child_frame_id, robot_pose):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = child_frame_id
        odom_msg.pose.pose.position.x = robot_pose[0] - 5.44
        odom_msg.pose.pose.position.y = robot_pose[1] - 5.44

        q = quaternion_from_euler(0, 0, robot_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_publisher_2.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = child_frame_id
        t.transform.translation.x = robot_pose[0] - 5.44
        t.transform.translation.y = robot_pose[1] - 5.44
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = turtlesim_pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()