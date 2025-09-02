#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.msg import Pose
from std_msgs.msg import Int64, Bool
from controller_interfaces.srv import SetMaxPizza, SetParam

class Killer(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.mouse_pose = np.array([0.0, 0.0])
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.robot_pose_1 = np.array([0.0, 0.0, 0.0])
        self.pizza_count = 0
        self.max_pizza = 5
        self.eat_state = 0
        self.eat_status = 0

        self.linear_kp = 3.0
        self.angular_kp = 7.0

        self.declare_parameter('sampling_frequency', 10.0)
        self.frequency = self.get_parameter('sampling_frequency').value
        self.declare_parameter('kill', 'XXX')
        self.killed_turtle = self.get_parameter('kill').value
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Pose, f'/{self.killed_turtle}/pose', self.pose_callback_1, 10)
        self.create_subscription(Bool, f'/{self.killed_turtle}/eat_status', self.eat_status_callback, 10)

        self.create_timer(1/self.frequency, self.timer_callback)

        self.eat_target_client = self.create_client(Kill, '/remove_turtle')
        self.set_param_service = self.create_service(SetParam, 'set_controller_param', self.set_param_callback)

    def timer_callback(self):
        if (self.eat_status == False):
            self.cmd_vel(0.0, 0.0)
            return
        
        self.eat_state = 0
        target = self.robot_pose_1
        diff_x = target[0] - self.robot_pose[0]
        diff_y = target[1] - self.robot_pose[1]
        d = np.sqrt(diff_x**2 + diff_y**2)
        theta_d = np.arctan2(diff_y, diff_x)
        e_theta = theta_d - self.robot_pose[2]
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        vx = self.linear_kp * d
        w = self.angular_kp * e_theta
        self.cmd_vel(vx, w)

        if d < 0.1 and abs(e_theta) < 0.1 and self.eat_state == 0:
            self.eat_target()

    def cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta
    
    def pose_callback_1(self, msg):
        self.robot_pose_1[0] = msg.x
        self.robot_pose_1[1] = msg.y
        self.robot_pose_1[2] = msg.theta

    def eat_target(self):
        eat_req = Kill.Request()
        eat_req.name = self.killed_turtle
        future = self.eat_target_client.call_async(eat_req)
        future.add_done_callback(self.eat_target_callback)

    def eat_target_callback(self, future):
        future.result()
        self.cmd_vel(0.0, 0.0)
        self.eat_state = 1

    def eat_status_callback(self, msg):
        self.eat_status = msg.data
    
    def set_param_callback(self, request:SetParam.Request, response:SetParam.Response):
        self.linear_kp = request.kp_linear.data
        self.angular_kp = request.kp_angular.data
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Killer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
