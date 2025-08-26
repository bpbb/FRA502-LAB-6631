#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64

class Eater(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.mouse_pose = np.array([0.0, 0.0])
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.target = np.array([0.0, 0.0])
        self.evade_target = []
        self.queue = []
        self.pizza_count = 0
        self.pizza_check = 0
        self.max_pizza = 5

        self.linear_kp = 3.0
        self.angular_kp = 7.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)

        self.create_timer(0.01, self.timer_callback)

        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

    def timer_callback(self):
        if not self.queue:
            if not self.evade_target:
                self.cmd_vel(0.0, 0.0)
                return
            else:
                self.target = self.evade_target
        else:
            self.target = self.queue[0]

        diff_x = self.target[0] - self.robot_pose[0]
        diff_y = self.target[1] - self.robot_pose[1]
        d = np.sqrt(diff_x**2 + diff_y**2)
        theta_d = np.arctan2(diff_y, diff_x)
        e_theta = theta_d - self.robot_pose[2]
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        vx = self.linear_kp * d
        w = self.angular_kp * e_theta
        self.cmd_vel(vx, w)

        if d < 0.1 and abs(e_theta) < 0.1:
            self.cmd_vel(0.0, 0.0)
            self.eat_pizza()

    def cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def mouse_position_callback(self, msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y

        if (self.pizza_count < self.max_pizza) and (self.pizza_check < self.max_pizza):
            self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1])
        else:
            self.evade_target = [self.mouse_pose[0], self.mouse_pose[1]]

    def spawn_pizza(self, x, y):
        pos_req = GivePosition.Request()
        pos_req.x = x
        pos_req.y = y
        self.queue.append([x, y])
        self.pizza_check += 1
        self.spawn_pizza_client.call_async(pos_req)

    def pizza_count_callback(self, msg):
        self.pizza_count = msg.data

    def eat_pizza(self):
        eat_req = Empty.Request()
        future = self.eat_pizza_client.call_async(eat_req)
        future.add_done_callback(self.eat_pizza_callback)

    def eat_pizza_callback(self, future):
        future.result()
        if self.queue:
            self.queue.pop(0)
        
    def goal_pose_callback(self, msg):
        x = msg.pose.position.x + 5.44
        y = msg.pose.position.y + 5.44
        
        if (self.pizza_count < self.max_pizza) and (self.pizza_check < self.max_pizza):
            self.queue.append([x, y])
        else:
            self.evade_target = [x, y]
        
def main(args=None):
    rclpy.init(args=args)
    node = Eater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
