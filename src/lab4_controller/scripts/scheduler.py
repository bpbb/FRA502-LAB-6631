#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import SetControlMode, GetRandomPose
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import numpy as np


class RobotSchedulerNode(Node):
    def __init__(self):
        super().__init__("robot_scheduler_node")

        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Service clients
        self.random_client = self.create_client(GetRandomPose, "get_random_pose")
        self.controller_client = self.create_client(SetControlMode, "controller_server")
        
        # Service server for mode requests
        self.robot_state_server = self.create_service(
            SetControlMode, "set_control_mode", self.robot_state_server_callback
        )
        
        # Publishers
        self.robot_state_pub = self.create_publisher(String, "current_state", 10)
        
        # Subscribe to controller status
        self.create_subscription(String, "/controller_status", self.controller_status_callback, 10)
        
        # State management
        self.current_state = "IDLE"

        # Robot parameters
        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2

        self.get_logger().info("="*60)
        self.get_logger().info("Robot Scheduler Node Started")
        self.get_logger().info("="*60)
        self.get_logger().info("Available modes:")
        self.get_logger().info("  - IK    : Inverse Kinematics mode")
        self.get_logger().info("  - AM    : Auto Mode (continuous random targets)")
        self.get_logger().info("  - TO_F  : Teleoperation - End Effector Frame")
        self.get_logger().info("  - TO_G  : Teleoperation - Global (World) Frame")
        self.get_logger().info("  - IDLE  : Idle mode")
        self.get_logger().info("="*60)

    def controller_status_callback(self, msg: String):
        status = msg.data
        
        if status == "TARGET_REACHED":
            if self.current_state == "AM":
                self.get_logger().info("AUTO target reached. Requesting next random pose...")
                self.req_random()
            elif self.current_state == "IK":
                self.get_logger().info("IK movement completed successfully")
                self.current_state = "IDLE"

    def req_random(self):
        self.get_logger().info("Requesting random pose from random_node")
        state_request = GetRandomPose.Request()
        state_request.trigger = True
        future = self.random_client.call_async(state_request)
        future.add_done_callback(self.callback_req_random)

    def callback_req_random(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Received random pose - x: {response.target_pose.pose.position.x:.3f}, "
                f"y: {response.target_pose.pose.position.y:.3f}, "
                f"z: {response.target_pose.pose.position.z:.3f}"
            )
            Pe = [
                float(response.target_pose.pose.position.x),
                float(response.target_pose.pose.position.y),
                float(response.target_pose.pose.position.z),
            ]

            if Pe is not None:
                self.get_logger().info(f"Sending AUTO command to controller with target: {Pe}")
                self.req_controller("AUTO", response.target_pose)
            else:
                self.get_logger().error("Could not get valid position from random node.")

        except Exception as e:
            self.get_logger().error(f"Error in callback_req_random: {e}")

    def req_controller(self, mode, target_pose):
        try:
            self.get_logger().info(f"Requesting controller mode: {mode}")
            controller_request = SetControlMode.Request()
            controller_request.mode_name = str(mode)
            controller_request.target_pose = target_pose

            future = self.controller_client.call_async(controller_request)
            future.add_done_callback(
                lambda f: self.get_logger().info(f"Controller responded: {f.result().message}")
            )
        except Exception as e:
            self.get_logger().error(f"Failed to request controller: {e}")
            return None

    def req_ik(self, request):
        try:
            self.get_logger().info(
                f"IK Request - x: {request.target_pose.pose.position.x:.3f}, "
                f"y: {request.target_pose.pose.position.y:.3f}, "
                f"z: {request.target_pose.pose.position.z:.3f}"
            )
            q = self.inverse_kinematic(
                float(request.target_pose.pose.position.x),
                float(request.target_pose.pose.position.y),
                float(request.target_pose.pose.position.z),
            )

            if q is not None:
                self.get_logger().info(f"✓ IK Solution found: q = [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}] rad")
                self.req_controller("IK", request.target_pose)
                return True, q
            else:
                self.get_logger().error("✗ IK Solution NOT found - target unreachable or out of workspace.")
                return False, None

        except Exception as e:
            self.get_logger().error(f"Error in req_ik: {e}")
            return False, None

    def robot_state_server_callback(self, request: SetControlMode.Request, response: SetControlMode.Response):
        self.get_logger().info(
            f"State change request: {str(self.current_state)} -> {str(request.mode_name)}"
        )
        
        if str(request.mode_name) == "AM":
            self.current_state = "AM"
            self.get_logger().info("AUTO mode activated - requesting first random pose")
            self.req_random()
            response.success = True
            response.message = "AUTO mode activated"

        elif str(request.mode_name) == "IK":
            ik_success, q_solution = self.req_ik(request)
            if ik_success:
                self.current_state = "IK"
                response.success = True
                response.message = "IK solution found. Robot moving to target."
                if q_solution is not None:
                    response.configuration_solution.name = ["joint_1", "joint_2", "joint_3"]
                    response.configuration_solution.position = q_solution.tolist()
            else:
                response.success = False
                response.message = "IK solution NOT found. Robot remains in current state."

        elif str(request.mode_name) == "TO_F":
            self.current_state = "TO_F"
            self.req_controller("TELEOP_F", request.target_pose)
            response.success = True
            response.message = "Teleoperation End-Effector Frame mode activated"
            self.get_logger().info("✓ TO_F: Teleoperation in End-Effector frame")

        elif str(request.mode_name) == "TO_G":
            self.current_state = "TO_G"
            self.req_controller("TELEOP_G", request.target_pose)
            response.success = True
            response.message = "Teleoperation Global Frame mode activated"
            self.get_logger().info("✓ TO_G: Teleoperation in World frame")

        elif str(request.mode_name) == "IDLE":
            self.current_state = "IDLE"
            self.req_controller("IDLE", request.target_pose)
            response.success = True
            response.message = "Returned to IDLE"
            self.get_logger().info("Returned to IDLE mode")

        else:
            response.success = False
            response.message = f"Unknown mode: {request.mode_name}"
            self.get_logger().error(f"Unknown mode requested: {request.mode_name}")

        return response

    def inverse_kinematic(self, x, y, z):
        distance_squared = x**2 + y**2 + (z - 0.2) ** 2
        distance = np.sqrt(distance_squared)
        
        if distance_squared < self.r_min**2:
            self.get_logger().error(f"Target too close: distance={distance:.3f} < r_min={self.r_min:.3f}")
            return None
        if distance_squared > self.r_max**2:
            self.get_logger().error(f"Target too far: distance={distance:.3f} > r_max={self.r_max:.3f}")
            return None

        robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )

        T_Position = SE3(x, y, z)
        
        initial_guesses = [
            [0, 0, 0],
            [0, pi/4, -pi/4],
            [pi/2, pi/4, 0],
            [-pi/2, pi/4, 0],
        ]
        
        for q0 in initial_guesses:
            ik_solution = robot.ikine_LM(
                T_Position, mask=[1, 1, 1, 0, 0, 0], joint_limits=False, q0=q0
            )

            if ik_solution.success:
                q_sol_ik_LM = ik_solution.q
                self.get_logger().info(f"IK converged with error: {ik_solution.residual:.6f}")
                return q_sol_ik_LM
        
        self.get_logger().error("Inverse kinematics failed to converge with all initial guesses.")
        return None

    def timer_callback(self):
        msg = String()
        msg.data = self.current_state
        self.robot_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()