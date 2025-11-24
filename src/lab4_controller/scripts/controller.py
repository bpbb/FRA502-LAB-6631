#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from interfaces.srv import SetControlMode
import numpy as np
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import time


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Service server for receiving commands from scheduler
        self.controller_server = self.create_service(
            SetControlMode, "controller_server", self.controller_server_callback
        )
        
        # Controller state (the only state we need to track)
        self.controller_state = "IDLE"

        # Status publisher (notifies scheduler when done)
        self.status_pub = self.create_publisher(String, "/controller_status", 10)

        # Teleoperation velocity input
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.tele_x = 0.0
        self.tele_y = 0.0
        self.tele_z = 0.0

        # TF for actual position feedback
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.endeff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.singularity_pub = self.create_publisher(String, '/singularity_alert', 1)

        # Control parameters
        self.kp = 2.0
        self.q = np.array([0.0, 0.0, 0.0])
        self.r_max = 0.53
        self.r_min = 0.03
        self.z_min = 0.02  # Minimum height above ground (2cm safety margin)
        
        # Target setpoints
        self.ik_setpoint = [0, 0, 0]
        self.random_setpoint = [0, 0, 0]
        self.auto_target_reached = False
        
        # Joint limits
        self.q_max = np.array([pi, pi, pi])
        self.q_min = np.array([-pi, -pi, -pi])
        self.q_mid = (self.q_max + self.q_min) / 2

        # Stagnation detection
        self.prev_error = None
        self.stagnation_counter = 0
        self.stagnation_threshold = 300
        self.error_improvement_threshold = 0.0001
        self.movement_start_time = None
        self.timeout_duration = 10.0
        self.stuck_in_singularity = False
        self.singularity_counter = 0
        self.singularity_stuck_threshold = 200
        self.movement_started = False

        # Singularity detection
        self.singularity_epsilon = 0.001
        self.near_singularity = False

        # Joint state publisher
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_state.position = [0.0, 0.0, 0.0]

        # Robot model
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )
        self.publish_joint_state(np.array([0.0, 0.0, 0.0]))
        self.get_logger().info("Controller started - Modes: IK, AUTO, TELEOP_F, TELEOP_G")

    def cmd_vel_callback(self, msg: Twist):
        self.tele_x = msg.linear.x
        self.tele_y = msg.linear.y
        self.tele_z = msg.linear.z

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def reset_stagnation_detection(self):
        self.prev_error = None
        self.stagnation_counter = 0
        self.movement_start_time = None
        self.stuck_in_singularity = False
        self.singularity_counter = 0
        self.movement_started = False
        self.near_singularity = False

    def controller_server_callback(self, request: SetControlMode.Request, response: SetControlMode.Response):
        self.controller_state = request.mode_name
        self.reset_stagnation_detection()

        if self.controller_state == "AUTO":
            target_z = float(request.target_pose.pose.position.z)
            if target_z < self.z_min:
                self.get_logger().error(f"AUTO target rejected: z={target_z:.3f}m below ground limit {self.z_min}m")
                response.success = False
                response.message = f"Target below ground (z={target_z:.3f}m)"
                self.controller_state = "IDLE"
                return response
            
            self.random_setpoint = [
                float(request.target_pose.pose.position.x),
                float(request.target_pose.pose.position.y),
                target_z,
            ]
            self.auto_target_reached = False
            self.movement_start_time = time.time()
            self.get_logger().info(f"AUTO mode: Target [{self.random_setpoint[0]:.3f}, {self.random_setpoint[1]:.3f}, {self.random_setpoint[2]:.3f}]")
            
        elif self.controller_state == "IK":
            target_z = float(request.target_pose.pose.position.z)
            if target_z < self.z_min:
                self.get_logger().error(f"IK target rejected: z={target_z:.3f}m below ground limit {self.z_min}m")
                response.success = False
                response.message = f"Target below ground (z={target_z:.3f}m)"
                self.controller_state = "IDLE"
                return response
            
            self.ik_setpoint = [
                float(request.target_pose.pose.position.x),
                float(request.target_pose.pose.position.y),
                target_z,
            ]
            self.movement_start_time = time.time()
            self.get_logger().info(f"IK mode: Target [{self.ik_setpoint[0]:.3f}, {self.ik_setpoint[1]:.3f}, {self.ik_setpoint[2]:.3f}]")
            
        elif self.controller_state == "TELEOP_F":
            self.get_logger().info("TELEOP_F mode: End-effector frame control activated")
            
        elif self.controller_state == "TELEOP_G":
            self.get_logger().info("TELEOP_G mode: World frame control activated")
        
        elif self.controller_state == "IDLE":
            self.get_logger().info("IDLE mode: Controller stopped")

        response.success = True
        response.message = f"Controller: {self.controller_state}"
        return response

    def publish_joint_state(self, positions):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        try:
            self.joint_state.position = positions.tolist()
        except:
            self.joint_state.position = positions
        self.joint_state_publisher.publish(self.joint_state)

    def get_position_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            position = transform.transform.translation
            rotation_quat = transform.transform.rotation
            
            rotation = R.from_quat([rotation_quat.x, rotation_quat.y, rotation_quat.z, rotation_quat.w])
            rotation_matrix = rotation.as_matrix()
            
            return np.array([position.x, position.y, position.z]), rotation_matrix
        except Exception as e:
            T = self.robot.fkine(self.q)
            return T.t, T.R

    def check_stagnation(self, current_error):
        if self.prev_error is None:
            self.prev_error = current_error
            return False
        
        if not self.movement_started:
            if self.prev_error - current_error > 0.001:
                self.movement_started = True
            self.prev_error = current_error
            return False
        
        error_reduction = self.prev_error - current_error
        if error_reduction < self.error_improvement_threshold:
            self.stagnation_counter += 1
        else:
            self.stagnation_counter = 0
        self.prev_error = current_error
        
        if self.movement_start_time is not None:
            elapsed = time.time() - self.movement_start_time
            if elapsed > self.timeout_duration:
                self.get_logger().error(f"TIMEOUT: {elapsed:.1f}s exceeded limit")
                return True
        
        if self.stagnation_counter >= self.stagnation_threshold:
            self.get_logger().error("STAGNATION: No progress detected")
            return True
        return False

    def check_singularity_stuck(self, s_min):
        if s_min < 0.005:
            self.singularity_counter += 1
            if self.singularity_counter >= self.singularity_stuck_threshold:
                if not self.stuck_in_singularity:
                    self.stuck_in_singularity = True
                    self.get_logger().error(f"STUCK IN SINGULARITY")
                return True
        else:
            self.singularity_counter = 0
            self.stuck_in_singularity = False
        return False

    def handle_stuck_situation(self):
        self.get_logger().error(f"STUCK in {self.controller_state}")
        self.publish_status("TARGET_REACHED")
        self.controller_state = "IDLE"
        self.reset_stagnation_detection()

    def detect_singularity(self, J):
        U, S, Vt = np.linalg.svd(J)
        s_min = np.min(S)
        is_near = s_min < self.singularity_epsilon
        return is_near, s_min

    def publish_singularity_warning(self):
        msg = String()
        msg.data = "WARNING: Approaching Singularity - Robot Stopped"
        self.singularity_pub.publish(msg)
        self.get_logger().warn("SINGULARITY WARNING: Robot stopped to avoid singularity")

    def control_to_pos(self, p_set):
        try:
            p_setpoint = np.array(p_set)
            p_now, r_now = self.get_position_from_tf()
            
            error = p_setpoint - p_now
            error_norm = np.linalg.norm(error)
            
            if error_norm <= 0.001:
                self.get_logger().info("TARGET REACHED")
                self.publish_status("TARGET_REACHED")
                self.controller_state = "IDLE"
                self.reset_stagnation_detection()
                return False
            
            if self.check_stagnation(error_norm):
                self.handle_stuck_situation()
                return False
            
            p_dot = self.kp * error
            q_dot, s_min = self.singularity_robust_inverse(p_dot)
            
            if self.check_singularity_stuck(s_min):
                self.handle_stuck_situation()
                return False
            
            q_dot_null = 0.5 * (self.q_mid - self.q)
            J = self.robot.jacob0(self.q)[0:3, :]
            J_pinv, _ = self.compute_singularity_robust_inverse(J)
            N = np.eye(3) - J_pinv @ J
            
            q_dot_total = q_dot + N @ q_dot_null
            q_new = self.q + q_dot_total / self.frequency
            q_new = np.clip(q_new, self.q_min, self.q_max)
            
            # Ground collision check - verify new position stays above ground
            T_new = self.robot.fkine(q_new)
            if T_new.t[2] < self.z_min:
                self.get_logger().warn(f"Ground collision prevented at z={T_new.t[2]:.3f}m")
                return True  # Don't update joints, but continue running
            
            self.q = q_new

            self.publish_joint_state(self.q)
            return True
        except Exception as e:
            self.get_logger().error(f"control_to_pos error: {e}")
            return False

    def teleop_control(self, frame_type="global"):
        try:
            v_cmd = np.array([self.tele_x, self.tele_y, self.tele_z])
            
            if np.linalg.norm(v_cmd) < 1e-6:
                return True
            
            p_now, r_now = self.get_position_from_tf()
            
            if frame_type == "end_effector":
                p_dot = r_now @ v_cmd
            else:
                p_dot = v_cmd
            
            J = self.robot.jacob0(self.q)[0:3, :]
            is_near_singularity, s_min = self.detect_singularity(J)
            
            if is_near_singularity:
                if not self.near_singularity:
                    self.near_singularity = True
                    self.publish_singularity_warning()
                return True
            else:
                self.near_singularity = False
            
            J_inv, _ = self.compute_singularity_robust_inverse(J)
            q_dot = J_inv @ p_dot
            
            q_dot_null = 0.5 * (self.q_mid - self.q)
            N = np.eye(3) - J_inv @ J
            q_dot_total = q_dot + N @ q_dot_null
            
            q_new = self.q + q_dot_total / self.frequency
            q_new = np.clip(q_new, self.q_min, self.q_max)
            
            # Ground collision check - verify new position stays above ground
            T_new = self.robot.fkine(q_new)
            if T_new.t[2] < self.z_min:
                self.get_logger().warn(f"Ground collision prevented at z={T_new.t[2]:.3f}m")
                return True  # Don't update joints, but continue running
            
            self.q = q_new
            
            self.publish_joint_state(self.q)
            return True
            
        except Exception as e:
            self.get_logger().error(f"teleop_control error: {e}")
            return False

    def singularity_robust_inverse(self, p_dot):
        J = self.robot.jacob0(self.q)[0:3, :]
        J_inv, s_min = self.compute_singularity_robust_inverse(J)
        return J_inv @ p_dot, s_min

    def compute_singularity_robust_inverse(self, J):
        U, S, Vt = np.linalg.svd(J)
        s_min = np.min(S)
        epsilon = 0.01
        lambda_max = 0.1
        
        if s_min < epsilon:
            lambda_sq = lambda_max * (1 - (s_min / epsilon)**2)
        else:
            lambda_sq = 0.0001
        
        J_dls = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(J.shape[0]))
        return J_dls, s_min

    def rviz_pub(self, pos):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'link_0'
        target.pose.position.x = float(pos[0])
        target.pose.position.y = float(pos[1])
        target.pose.position.z = float(pos[2])
        self.target_pub.publish(target)

        p_now, _ = self.get_position_from_tf()
        endeff = PoseStamped()
        endeff.header.stamp = self.get_clock().now().to_msg()
        endeff.header.frame_id = "link_0"
        endeff.pose.position.x = float(p_now[0])
        endeff.pose.position.y = float(p_now[1])
        endeff.pose.position.z = float(p_now[2])
        self.endeff_pub.publish(endeff)

    def timer_callback(self):
        try:
            if self.controller_state == "AUTO":
                if self.random_setpoint != [0, 0, 0]:
                    self.control_to_pos(self.random_setpoint)
                    self.rviz_pub(self.random_setpoint)
                    
            elif self.controller_state == "IK":
                if self.ik_setpoint != [0, 0, 0]:
                    self.control_to_pos(self.ik_setpoint)
                    self.rviz_pub(self.ik_setpoint)
                    
            elif self.controller_state == "TELEOP_F":
                self.teleop_control(frame_type="end_effector")
                p_now, _ = self.get_position_from_tf()
                self.rviz_pub(p_now)
                
            elif self.controller_state == "TELEOP_G":
                self.teleop_control(frame_type="global")
                p_now, _ = self.get_position_from_tf()
                self.rviz_pub(p_now)
                
        except Exception as e:
            self.get_logger().error(f"timer_callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()